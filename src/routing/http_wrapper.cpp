/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <cctype>
#include <charconv>
#include <chrono>
#include <format>
#include <optional>
#include <tuple>
#include <utility>

#include <asio.hpp>
#include <asio/ssl.hpp>

#include "routing/http_wrapper.h"

using asio::ip::tcp;

namespace vroom::routing {

const std::string HttpWrapper::HTTPS_PORT = "443";

HttpWrapper::HttpWrapper(const std::string& profile,
                         Server server,
                         std::string matrix_service,
                         std::string matrix_durations_key,
                         std::string matrix_distances_key,
                         std::string route_service,
                         std::string routing_args)
  : Wrapper(profile),
    _server(std::move(server)),
    _matrix_service(std::move(matrix_service)),
    _matrix_durations_key(std::move(matrix_durations_key)),
    _matrix_distances_key(std::move(matrix_distances_key)),
    _route_service(std::move(route_service)),
    _routing_args(std::move(routing_args)) {
}

namespace {

// Time allowed to establish a connection, including TLS handshake.
constexpr auto CONNECT_TIMEOUT = std::chrono::seconds(30);

// Maximum time without any progress while sending a request or
// receiving a response. This includes waiting for the server to
// compute the response, but slow transfers that keep progressing are
// not interrupted.
constexpr auto IDLE_TIMEOUT = std::chrono::minutes(10);

constexpr std::size_t READ_BUFFER_SIZE = std::size_t{1} << 16;

// Maximum size of response excerpts reported in error messages.
constexpr std::size_t MAX_EXCERPT_SIZE = 200;

constexpr std::string_view CRLF = "\r\n";
constexpr std::string_view HEADERS_END = "\r\n\r\n";

using Clock = std::chrono::steady_clock;

std::string excerpt(std::string_view s) {
  if (s.size() <= MAX_EXCERPT_SIZE) {
    return std::string(s);
  }
  return std::string(s.substr(0, MAX_EXCERPT_SIZE)) + "...";
}

bool iequals(std::string_view a, std::string_view b) {
  return std::ranges::equal(a, b, [](char x, char y) {
    return std::tolower(static_cast<unsigned char>(x)) ==
           std::tolower(static_cast<unsigned char>(y));
  });
}

std::string_view trim(std::string_view s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
    s.remove_prefix(1);
  }
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) {
    s.remove_suffix(1);
  }
  return s;
}

// Run pending asynchronous operation until completion or deadline,
// in which case the socket is closed to abort the operation. Return
// false on timeout.
bool run_until(asio::io_context& io_context,
               tcp::socket& socket,
               Clock::time_point deadline) {
  io_context.restart();
  io_context.run_until(deadline);
  if (!io_context.stopped()) {
    std::error_code ignored;
    socket.close(ignored);
    io_context.run();
    return false;
  }
  return true;
}

void connect(asio::io_context& io_context,
             tcp::socket& socket,
             const Server& server,
             Clock::time_point deadline) {
  tcp::resolver r(io_context);
  std::error_code error;
  const auto endpoints = r.resolve(server.host, server.port, error);
  if (error) {
    throw RoutingException("Failed to resolve " + server.host + ": " +
                           error.message());
  }

  asio::async_connect(socket,
                      endpoints,
                      [&error](const std::error_code& e, const tcp::endpoint&) {
                        error = e;
                      });
  if (!run_until(io_context, socket, deadline)) {
    throw RoutingException("Timed out connecting to " + server.host + ":" +
                           server.port);
  }
  if (error) {
    throw RoutingException("Failed to connect to " + server.host + ":" +
                           server.port + ": " + error.message());
  }
}

// Parse chunked-encoded data stored in raw from start. Return false
// if data is incomplete or malformed. If decode is true, chunk data
// is moved in place to form a contiguous body starting at start, and
// the decoded body size is stored in body_size.
bool parse_chunked(std::string& raw,
                   std::size_t start,
                   bool decode,
                   std::size_t& body_size) {
  std::size_t read = start;
  std::size_t write = start;
  for (;;) {
    const auto line_end = raw.find(CRLF, read);
    if (line_end == std::string::npos) {
      return false;
    }
    // Chunk size, possibly followed by ignored chunk extensions.
    std::size_t chunk_size = 0;
    const char* const line_start = raw.data() + read;
    const auto [ptr, ec] =
      std::from_chars(line_start, raw.data() + line_end, chunk_size, 16);
    if (ec != std::errc() || ptr == line_start) {
      return false;
    }
    read = line_end + CRLF.size();

    if (chunk_size == 0) {
      // Last chunk, skip optional trailer fields up to empty line.
      for (;;) {
        const auto trailer_end = raw.find(CRLF, read);
        if (trailer_end == std::string::npos) {
          return false;
        }
        if (trailer_end == read) {
          break;
        }
        read = trailer_end + CRLF.size();
      }
      break;
    }

    if (chunk_size > raw.size() - read ||
        raw.size() - read - chunk_size < CRLF.size() ||
        std::string_view(raw).substr(read + chunk_size, CRLF.size()) != CRLF) {
      return false;
    }
    if (decode) {
      std::copy(raw.begin() + read,
                raw.begin() + read + chunk_size,
                raw.begin() + write);
    }
    write += chunk_size;
    read += chunk_size + CRLF.size();
  }

  if (decode) {
    body_size = write - start;
  }
  return true;
}

// Parse status line and relevant headers in raw response.
void parse_headers(HttpResponse& response,
                   std::optional<std::size_t>& content_length,
                   bool& chunked) {
  const std::string_view headers =
    std::string_view(response.raw).substr(0, response.body_start);

  auto line_end = headers.find(CRLF);
  const auto status_line = headers.substr(0, line_end);
  // Expecting e.g. "HTTP/1.1 200 OK".
  constexpr std::size_t status_start = 9;
  constexpr std::size_t status_size = 3;
  if (!status_line.starts_with("HTTP/") ||
      status_line.size() < status_start + status_size ||
      std::from_chars(status_line.data() + status_start,
                      status_line.data() + status_start + status_size,
                      response.status)
          .ec != std::errc()) {
    throw RoutingException("Invalid routing response: " + excerpt(status_line));
  }

  while (line_end != std::string_view::npos) {
    const auto line_start = line_end + CRLF.size();
    line_end = headers.find(CRLF, line_start);
    const auto line = headers.substr(line_start, line_end - line_start);

    const auto colon = line.find(':');
    if (colon == std::string_view::npos) {
      continue;
    }
    const auto name = trim(line.substr(0, colon));
    const auto value = trim(line.substr(colon + 1));

    if (iequals(name, "Transfer-Encoding")) {
      // Chunked has to be the last applied transfer coding.
      constexpr std::string_view chunked_coding = "chunked";
      chunked = value.size() >= chunked_coding.size() &&
                iequals(value.substr(value.size() - chunked_coding.size()),
                        chunked_coding);
    } else if (iequals(name, "Content-Length")) {
      std::size_t length = 0;
      const auto [ptr, ec] =
        std::from_chars(value.data(), value.data() + value.size(), length);
      if (ec != std::errc() || ptr != value.data() + value.size()) {
        throw RoutingException("Invalid Content-Length in routing response.");
      }
      content_length = length;
    }
  }
}

template <typename Stream>
HttpResponse exchange(asio::io_context& io_context,
                      Stream& stream,
                      tcp::socket& socket,
                      const std::string& query,
                      const Server& server,
                      std::size_t max_response_size) {
  const std::string target = server.host + ":" + server.port;

  std::error_code error;
  asio::async_write(stream,
                    asio::buffer(query),
                    [&error](const std::error_code& e, std::size_t) {
                      error = e;
                    });
  if (!run_until(io_context, socket, Clock::now() + IDLE_TIMEOUT)) {
    throw RoutingException("Timed out sending request to " + target);
  }
  if (error) {
    throw RoutingException("Failed to send request to " + target + ": " +
                           error.message());
  }

  HttpResponse response;
  std::string& raw = response.raw;
  bool has_headers = false;
  std::optional<std::size_t> content_length;
  bool chunked = false;
  std::size_t ignored_size = 0;

  std::vector<char> buf(READ_BUFFER_SIZE);
  for (;;) {
    std::size_t len = 0;
    stream.async_read_some(asio::buffer(buf),
                           [&error, &len](const std::error_code& e,
                                          std::size_t n) {
                             error = e;
                             len = n;
                           });
    if (!run_until(io_context, socket, Clock::now() + IDLE_TIMEOUT)) {
      throw RoutingException("Timed out reading response from " + target);
    }

    if (len > max_response_size - raw.size()) {
      throw RoutingException(
        std::format("Routing response from {} exceeds {} bytes.",
                    target,
                    max_response_size));
    }
    const auto previous_size = raw.size();
    raw.append(buf.data(), len);

    if (!has_headers) {
      const auto search_start = previous_size < HEADERS_END.size()
                                  ? 0
                                  : previous_size - HEADERS_END.size();
      if (const auto headers_end = raw.find(HEADERS_END, search_start);
          headers_end != std::string::npos) {
        has_headers = true;
        response.body_start = headers_end + HEADERS_END.size();
        parse_headers(response, content_length, chunked);
        if (content_length.has_value() && !chunked &&
            content_length.value() > max_response_size - response.body_start) {
          throw RoutingException(
            std::format("Routing response from {} exceeds {} bytes.",
                        target,
                        max_response_size));
        }
      }
    }

    // Stop as soon as a complete body is received, without relying
    // on the server closing the connection.
    if (has_headers && !chunked && content_length.has_value() &&
        raw.size() - response.body_start >= content_length.value()) {
      break;
    }
    if (has_headers && chunked && raw.ends_with(HEADERS_END) &&
        parse_chunked(raw, response.body_start, false, ignored_size)) {
      break;
    }

    if (error == asio::error::eof ||
        error == asio::ssl::error::stream_truncated) {
      // Connection closed, possibly without TLS close_notify. Body
      // completeness is checked below.
      break;
    }
    if (error) {
      throw RoutingException("Failed to read response from " + target + ": " +
                             error.message());
    }
  }

  if (!has_headers) {
    throw RoutingException("Invalid routing response: " + excerpt(raw));
  }

  if (chunked) {
    if (!parse_chunked(raw, response.body_start, true, response.body_size)) {
      throw RoutingException("Malformed or truncated chunked response from " +
                             target);
    }
  } else if (content_length.has_value()) {
    if (raw.size() - response.body_start < content_length.value()) {
      throw RoutingException("Truncated routing response from " + target);
    }
    response.body_size = content_length.value();
  } else {
    // Body delimited by connection close.
    response.body_size = raw.size() - response.body_start;
  }

  return response;
}

} // namespace

std::string_view HttpResponse::status_line() const {
  const std::string_view s(raw);
  return s.substr(0, s.find(CRLF));
}

bool HttpResponse::success() const {
  constexpr unsigned HTTP_OK = 200;
  constexpr unsigned HTTP_MULTIPLE_CHOICES = 300;
  return HTTP_OK <= status && status < HTTP_MULTIPLE_CHOICES;
}

HttpResponse
HttpWrapper::send_then_receive(const std::string& query,
                               std::size_t max_response_size) const {
  asio::io_context io_context;
  tcp::socket s(io_context);

  connect(io_context, s, _server, Clock::now() + CONNECT_TIMEOUT);

  return exchange(io_context, s, s, query, _server, max_response_size);
}

HttpResponse
HttpWrapper::ssl_send_then_receive(const std::string& query,
                                   std::size_t max_response_size) const {
  asio::io_context io_context;

  asio::ssl::context ctx(asio::ssl::context::method::tls_client);
  std::error_code error;
  ctx.set_default_verify_paths(error);
  if (error) {
    throw RoutingException("Failed to load default TLS certificates: " +
                           error.message());
  }
  ctx.set_verify_mode(asio::ssl::verify_peer);

  asio::ssl::stream<tcp::socket> ssock(io_context, ctx);

  // Check that server certificate matches host.
#if ASIO_VERSION >= 101601
  ssock.set_verify_callback(asio::ssl::host_name_verification(_server.host));
#else
  ssock.set_verify_callback(asio::ssl::rfc2818_verification(_server.host));
#endif

  // Set SNI, not allowed for IP addresses.
  std::ignore = asio::ip::make_address(_server.host, error);
  if (error) {
    if (SSL_set_tlsext_host_name(ssock.native_handle(), // NOLINT
                                 _server.host.c_str()) != 1) {
      throw RoutingException("Failed to set TLS server name for " +
                             _server.host);
    }
  }

  const auto connect_deadline = Clock::now() + CONNECT_TIMEOUT;
  connect(io_context, ssock.next_layer(), _server, connect_deadline);

  ssock.async_handshake(asio::ssl::stream_base::handshake_type::client,
                        [&error](const std::error_code& e) { error = e; });
  if (!run_until(io_context, ssock.next_layer(), connect_deadline)) {
    throw RoutingException("Timed out during TLS handshake with " +
                           _server.host + ":" + _server.port);
  }
  if (error) {
    throw RoutingException("TLS handshake with " + _server.host + ":" +
                           _server.port + " failed: " + error.message());
  }

  return exchange(io_context,
                  ssock,
                  ssock.next_layer(),
                  query,
                  _server,
                  max_response_size);
}

HttpResponse HttpWrapper::run_query(const std::string& query,
                                    std::size_t nb_locations) const {
  // Bound response size based on expected content: a fixed allowance
  // plus room for a full matrix (or route geometry) for the requested
  // number of locations.
  constexpr std::size_t BASE_RESPONSE_SIZE = std::size_t{256} << 20;
  constexpr std::size_t BYTES_PER_LOCATION_PAIR = 64;
  const std::size_t max_response_size =
    BASE_RESPONSE_SIZE + BYTES_PER_LOCATION_PAIR * nb_locations * nb_locations;

  return (_server.port == HTTPS_PORT)
           ? ssl_send_then_receive(query, max_response_size)
           : send_then_receive(query, max_response_size);
}

void HttpWrapper::parse_response(rapidjson::Document& json_result,
                                 const HttpResponse& response,
                                 const std::vector<Location>& locs,
                                 const std::string& service) const {
  // Parse in place from the response buffer, no copy required.
  const auto body = response.body();
  json_result.Parse(body.data(), body.size());

  if (json_result.HasParseError() || !json_result.IsObject()) {
    if (!response.success()) {
      throw RoutingException("Routing server error: " +
                             excerpt(response.status_line()));
    }
    throw RoutingException("Failed to parse routing response: " +
                           excerpt(body));
  }

  // Error responses usually come with a non-2xx status and a JSON
  // body describing the problem.
  this->check_response(json_result, locs, service);

  if (!response.success()) {
    throw RoutingException("Routing server error: " +
                           excerpt(response.status_line()));
  }
}

const rapidjson::Value& HttpWrapper::get_member(const rapidjson::Value& value,
                                                const char* key) {
  if (!value.IsObject()) {
    throw RoutingException(std::string("Invalid routing response, expected "
                                       "object containing ") +
                           key + ".");
  }
  const auto member = value.FindMember(key);
  if (member == value.MemberEnd()) {
    throw RoutingException(std::string("Missing ") + key +
                           " in routing response.");
  }
  return member->value;
}

const rapidjson::Value& HttpWrapper::get_array(const rapidjson::Value& value,
                                               const char* key) {
  const auto& array = get_member(value, key);
  if (!array.IsArray()) {
    throw RoutingException(std::string("Invalid ") + key +
                           " in routing response, expected array.");
  }
  return array;
}

const rapidjson::Value& HttpWrapper::get_first(const rapidjson::Value& value,
                                               const char* key) {
  const auto& array = get_array(value, key);
  if (array.Empty()) {
    throw RoutingException(std::string("Empty ") + key +
                           " in routing response.");
  }
  return array[0];
}

double HttpWrapper::get_number(const rapidjson::Value& value, const char* key) {
  const auto& number = get_member(value, key);
  if (!number.IsNumber()) {
    throw RoutingException(std::string("Invalid ") + key +
                           " in routing response, expected number.");
  }
  return number.GetDouble();
}

std::string HttpWrapper::get_string(const rapidjson::Value& value,
                                    const char* key) {
  const auto& str = get_member(value, key);
  if (!str.IsString()) {
    throw RoutingException(std::string("Invalid ") + key +
                           " in routing response, expected string.");
  }
  return std::string(str.GetString(), str.GetStringLength());
}

Matrices HttpWrapper::get_matrices(const std::vector<Location>& locs) const {
  const std::string query = this->build_query(locs, _matrix_service);
  const auto response = this->run_query(query, locs.size());

  // Expected matrix size.
  const std::size_t m_size = locs.size();

  rapidjson::Document json_result;
  this->parse_response(json_result, response, locs, _matrix_service);

  const auto& durations = get_array(json_result, _matrix_durations_key.c_str());
  const auto& distances = get_array(json_result, _matrix_distances_key.c_str());
  if (durations.Size() != m_size || distances.Size() != m_size) {
    throw RoutingException("Unexpected matrix size in routing response.");
  }

  // Build matrices while checking for unfound routes ('null' values)
  // to avoid unexpected behavior.
  Matrices m(m_size);

  std::vector<unsigned> nb_unfound_from_loc(m_size, 0);
  std::vector<unsigned> nb_unfound_to_loc(m_size, 0);

  for (rapidjson::SizeType i = 0; i < m_size; ++i) {
    const auto& duration_line = durations[i];
    const auto& distance_line = distances[i];
    if (!duration_line.IsArray() || duration_line.Size() != m_size ||
        !distance_line.IsArray() || distance_line.Size() != m_size) {
      throw RoutingException("Unexpected matrix size in routing response.");
    }
    for (rapidjson::SizeType j = 0; j < m_size; ++j) {
      if (duration_value_is_null(duration_line[j]) ||
          distance_value_is_null(distance_line[j])) {
        // No route found between i and j. Just storing info as we
        // don't know yet which location is responsible between i
        // and j.
        ++nb_unfound_from_loc[i];
        ++nb_unfound_to_loc[j];
      } else {
        m.durations[i][j] = get_duration_value(duration_line[j]);
        m.distances[i][j] = get_distance_value(distance_line[j]);
      }
    }
  }

  check_unfound(locs, nb_unfound_from_loc, nb_unfound_to_loc);

  return m;
}

void HttpWrapper::update_sparse_matrix(const std::vector<Location>& route_locs,
                                       Matrices& m,
                                       std::mutex& matrix_m,
                                       std::string& vehicle_geometry) const {
  const std::string query = this->build_query(route_locs, _route_service);

  const auto response = this->run_query(query, route_locs.size());

  rapidjson::Document json_result;
  this->parse_response(json_result, response, route_locs, _route_service);

  const auto& legs = get_legs(json_result);
  if (legs.Size() != route_locs.size() - 1) {
    throw RoutingException("Unexpected number of legs in routing response.");
  }

  for (rapidjson::SizeType i = 0; i < legs.Size(); ++i) {
    const auto duration = get_leg_duration(legs[i]);
    const auto distance = get_leg_distance(legs[i]);

    const std::scoped_lock<std::mutex> lock(matrix_m);
    m.durations[route_locs[i].index()][route_locs[i + 1].index()] = duration;
    m.distances[route_locs[i].index()][route_locs[i + 1].index()] = distance;
  }

  vehicle_geometry = get_geometry(json_result);
};

void HttpWrapper::add_geometry(Route& route) const {
  // Ordering locations for the given steps, excluding
  // breaks.
  std::vector<Location> non_break_locations;
  non_break_locations.reserve(route.steps.size());

  for (const auto& step : route.steps) {
    if (step.step_type != STEP_TYPE::BREAK) {
      assert(step.location.has_value());
      non_break_locations.push_back(step.location.value());
    }
  }
  assert(!non_break_locations.empty());

  const std::string query = build_query(non_break_locations, _route_service);

  const auto response = this->run_query(query, non_break_locations.size());

  rapidjson::Document json_result;
  this->parse_response(json_result,
                       response,
                       non_break_locations, // not supposed to be used
                       _route_service);

  if (get_legs(json_result).Size() != non_break_locations.size() - 1) {
    throw RoutingException("Unexpected number of legs in routing response.");
  }

  route.geometry = get_geometry(json_result);
}

} // namespace vroom::routing
