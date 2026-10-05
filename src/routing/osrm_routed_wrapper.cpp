/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <charconv>

#include "routing/osrm_routed_wrapper.h"

namespace vroom::routing {

OsrmRoutedWrapper::OsrmRoutedWrapper(const std::string& profile,
                                     const Server& server)
  : HttpWrapper(profile,
                server,
                "table",
                "durations",
                "distances",
                "route",
                "alternatives=false&steps=false&overview=full&continue_"
                "straight=false") {
}

std::string
OsrmRoutedWrapper::build_query(const std::vector<Location>& locations,
                               const std::string& service) const {
  // Building query for osrm-routed
  std::string query = "GET /" + _server.path + service;

  query += "/v1/" + profile + "/";

  // Build query part for snapping restriction.
  std::string radiuses = "radiuses=";
  radiuses.reserve(radiuses.size() +
                   locations.size() *
                     (DEFAULT_OSRM_SNAPPING_RADIUS.size() + 1));

  // Adding locations and radiuses values.
  for (auto const& location : locations) {
    query += std::format("{:.6f},{:.6f};", location.lon(), location.lat());
    radiuses += DEFAULT_OSRM_SNAPPING_RADIUS + ";";
  }
  // Remove trailing ';'.
  query.pop_back();
  radiuses.pop_back();

  if (service == _route_service) {
    query += "?" + _routing_args;
  } else {
    assert(service == _matrix_service);
    query += "?annotations=duration,distance";
  }
  query += "&" + radiuses;

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: close\r\n\r\n";

  return query;
}

void OsrmRoutedWrapper::check_response(const rapidjson::Document& json_result,
                                       const std::vector<Location>& locs,
                                       const std::string&) const {
  const std::string code = get_string(json_result, "code");
  if (code != "Ok") {
    std::string message = code;
    if (json_result.HasMember("message") && json_result["message"].IsString()) {
      message = json_result["message"].GetString();
    }

    if (const std::string snapping_error_base =
          "Could not find a matching segment for coordinate ";
        code == "NoSegment" && message.starts_with(snapping_error_base)) {
      std::size_t error_loc = 0;
      const char* const index_start =
        message.data() + snapping_error_base.size();
      const char* const index_end = message.data() + message.size();
      if (const auto [ptr, ec] =
            std::from_chars(index_start, index_end, error_loc);
          ec == std::errc() && ptr != index_start && error_loc < locs.size()) {
        const auto coordinates = std::format("[{:.6f},{:.6f}]",
                                             locs[error_loc].lon(),
                                             locs[error_loc].lat());
        throw RoutingException("Could not find route near location " +
                               coordinates);
      }
    }

    // Other error in response.
    throw RoutingException(message);
  }
}

const rapidjson::Value&
OsrmRoutedWrapper::get_legs(const rapidjson::Value& result) const {
  return get_array(get_first(result, "routes"), "legs");
}

} // namespace vroom::routing
