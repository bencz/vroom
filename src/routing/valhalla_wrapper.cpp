/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cctype>
#include <cstdint>
#include <string_view>

#include "../../include/polylineencoder/src/polylineencoder.h"

#include "routing/valhalla_wrapper.h"
#include "utils/helpers.h"

namespace vroom::routing {

constexpr unsigned km_to_m = 1000;
constexpr unsigned polyline_precision = 5;
constexpr unsigned valhalla_polyline_precision = 6;

namespace {

// Percent-encode all characters but unreserved ones (RFC 3986).
std::string url_encode(std::string_view value) {
  std::string encoded;
  encoded.reserve(value.size());
  for (const char c : value) {
    if (std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '-' ||
        c == '.' || c == '_' || c == '~') {
      encoded += c;
    } else {
      encoded += std::format("%{:02X}", static_cast<unsigned char>(c));
    }
  }
  return encoded;
}

// Check that an encoded polyline can be safely decoded: only valid
// characters, values fitting in 32 bits and decoded coordinates
// within valid ranges.
bool is_valid_polyline(std::string_view polyline) {
  constexpr char min_char = 63;
  constexpr char max_char = 126;
  constexpr int chunk_bits = 5;
  constexpr int max_chunks = 7; // Enough for 32-bit values.
  constexpr int continuation_bit = 0x20;
  constexpr int chunk_mask = 0x1f;
  constexpr std::int64_t max_lat = 90'000'000;
  constexpr std::int64_t max_lon = 180'000'000;

  std::int64_t lat = 0;
  std::int64_t lon = 0;
  bool is_lat = true;
  std::size_t i = 0;
  while (i < polyline.size()) {
    std::uint64_t result = 0;
    int nb_chunks = 0;
    int chunk = 0;
    do {
      if (i == polyline.size() || nb_chunks == max_chunks) {
        return false;
      }
      const char c = polyline[i++];
      if (c < min_char || max_char < c) {
        return false;
      }
      chunk = c - min_char;
      result |= static_cast<std::uint64_t>(chunk & chunk_mask)
                << (chunk_bits * nb_chunks);
      ++nb_chunks;
    } while ((chunk & continuation_bit) != 0);

    // Decoded value only uses the lower 32 bits.
    const auto value = static_cast<std::uint32_t>(result);
    const auto delta = static_cast<std::int64_t>(
      static_cast<std::int32_t>((value & 1) != 0 ? ~value : value) >> 1);

    if (is_lat) {
      lat += delta;
      if (lat < -max_lat || max_lat < lat) {
        return false;
      }
    } else {
      lon += delta;
      if (lon < -max_lon || max_lon < lon) {
        return false;
      }
    }
    is_lat = !is_lat;
  }

  // Coordinates come in pairs.
  return is_lat;
}

} // namespace

ValhallaWrapper::ValhallaWrapper(const std::string& profile,
                                 const Server& server)
  : HttpWrapper(profile,
                server,
                "sources_to_targets",
                "sources_to_targets",
                "sources_to_targets",
                "route",
                R"("directions_type":"none")") {
}

std::string ValhallaWrapper::get_matrix_query(
  const std::vector<Location>& locations) const {
  // Building matrix query for Valhalla.
  std::string query = "GET /" + _server.path + _matrix_service + "?json=";

  // List locations.
  std::string all_locations;
  for (auto const& location : locations) {
    all_locations += std::format(R"({{"lon":{:.6f},"lat":{:.6f}}},)",
                                 location.lon(),
                                 location.lat());
  }
  all_locations.pop_back(); // Remove trailing ','.

  std::string json = "{\"sources\":[" + all_locations;
  json += "],\"targets\":[" + all_locations;
  json += R"(],"costing":")" + profile + "\"}";

  query += url_encode(json);

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: Close\r\n\r\n";

  return query;
}

std::string
ValhallaWrapper::get_route_query(const std::vector<Location>& locations) const {
  // Building route query for Valhalla.
  std::string query = "GET /" + _server.path + _route_service + "?json=";

  std::string json = "{\"locations\":[";
  for (auto const& location : locations) {
    json += std::format(R"({{"lon":{:.6f},"lat":{:.6f},"type":"break"}},)",
                        location.lon(),
                        location.lat());
  }
  json.pop_back(); // Remove trailing ','.

  json += R"(],"costing":")" + profile + "\"";
  json += "," + _routing_args;
  json += "}";

  query += url_encode(json);

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: Close\r\n\r\n";

  return query;
}

std::string ValhallaWrapper::build_query(const std::vector<Location>& locations,
                                         const std::string& service) const {
  assert(service == _matrix_service || service == _route_service);

  return (service == _matrix_service) ? get_matrix_query(locations)
                                      : get_route_query(locations);
}

void ValhallaWrapper::check_response(const rapidjson::Document& json_result,
                                     const std::vector<Location>&,
                                     const std::string& service) const {
  assert(service == _matrix_service || service == _route_service);

  if (constexpr unsigned HTTP_OK = 200;
      json_result.HasMember("status_code") &&
      json_result["status_code"].IsUint() &&
      json_result["status_code"].GetUint() != HTTP_OK) {
    // Valhalla responses seem to only have a status_code key when a
    // problem is encountered. In that case it's not really clear what
    // keys can be expected so we're playing guesses. This happens
    // e.g. when requested matrix/route size goes over the server
    // limit.
    const std::string service_str =
      (service == _route_service) ? "route" : "matrix";
    std::string error = "Valhalla " + service_str + " error (";

    if (json_result.HasMember("error") && json_result["error"].IsString()) {
      error += json_result["error"].GetString();
      error += ").";
    }
    throw RoutingException(error);
  }

  if (service == _route_service) {
    const auto& trip = get_member(json_result, "trip");
    if (const auto& status = get_member(trip, "status");
        !status.IsInt() || status.GetInt() != 0) {
      if (trip.HasMember("status_message") &&
          trip["status_message"].IsString()) {
        throw RoutingException(std::string(trip["status_message"].GetString()));
      }
      throw RoutingException("Valhalla route error.");
    }
  }
}

bool ValhallaWrapper::duration_value_is_null(
  const rapidjson::Value& matrix_entry) const {
  return get_member(matrix_entry, "time").IsNull();
}

bool ValhallaWrapper::distance_value_is_null(
  const rapidjson::Value& matrix_entry) const {
  return get_member(matrix_entry, "distance").IsNull();
}

UserDuration ValhallaWrapper::get_duration_value(
  const rapidjson::Value& matrix_entry) const {
  return utils::round<UserDuration>(get_number(matrix_entry, "time"));
}

UserDistance ValhallaWrapper::get_distance_value(
  const rapidjson::Value& matrix_entry) const {
  return utils::round<UserDistance>(km_to_m *
                                    get_number(matrix_entry, "distance"));
}

const rapidjson::Value&
ValhallaWrapper::get_legs(const rapidjson::Value& result) const {
  return get_array(get_member(result, "trip"), "legs");
}

UserDuration
ValhallaWrapper::get_leg_duration(const rapidjson::Value& leg) const {
  return utils::round<UserDuration>(
    get_number(get_member(leg, "summary"), "time"));
}

UserDistance
ValhallaWrapper::get_leg_distance(const rapidjson::Value& leg) const {
  return utils::round<UserDistance>(
    km_to_m * get_number(get_member(leg, "summary"), "length"));
}

std::string ValhallaWrapper::get_geometry(rapidjson::Value& result) const {
  // Valhalla returns one polyline per route leg so we need to merge
  // them. Also taking the opportunity to adjust the encoding
  // precision as Valhalla uses 6 and we use 5 based on other routing
  // engine output. Note: getting directly a single polyline (e.g. by
  // not sending type=break for the route request) is not an option
  // since we have to force allowing u-turns in order to get a
  // geometry that is consistent with the time/distance values in
  // matrices.

  const auto& legs = get_legs(result);
  if (legs.Empty()) {
    throw RoutingException("Empty legs in routing response.");
  }

  auto get_shape = [](const rapidjson::Value& leg) {
    auto shape = get_string(leg, "shape");
    if (!is_valid_polyline(shape)) {
      throw RoutingException("Invalid shape in routing response.");
    }
    return shape;
  };

  auto full_polyline =
    gepaf::PolylineEncoder<valhalla_polyline_precision>::decode(
      get_shape(legs[0]));

  for (rapidjson::SizeType i = 1; i < legs.Size(); ++i) {
    auto decoded_pts =
      gepaf::PolylineEncoder<valhalla_polyline_precision>::decode(
        get_shape(legs[i]));

    if (!full_polyline.empty()) {
      full_polyline.pop_back();
    }
    full_polyline.insert(full_polyline.end(),
                         std::make_move_iterator(decoded_pts.begin()),
                         std::make_move_iterator(decoded_pts.end()));
  }

  gepaf::PolylineEncoder<polyline_precision> encoder;
  for (const auto& p : full_polyline) {
    encoder.addPoint(p.latitude(), p.longitude());
  }

  return encoder.encode();
}

} // namespace vroom::routing
