#ifndef HTTP_WRAPPER_H
#define HTTP_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include <string_view>

#include "../include/rapidjson/include/rapidjson/document.h"

#include "routing/wrapper.h"
#include "structures/typedefs.h"
#include "utils/helpers.h"

namespace vroom::routing {

// Raw HTTP response, body is stored (de-chunked if required) in raw
// starting at body_start.
struct HttpResponse {
  std::string raw;
  unsigned status = 0;
  std::size_t body_start = 0;
  std::size_t body_size = 0;

  std::string_view body() const {
    return std::string_view(raw).substr(body_start, body_size);
  }

  std::string_view status_line() const;

  bool success() const;
};

class HttpWrapper : public Wrapper {
private:
  HttpResponse send_then_receive(const std::string& query,
                                 std::size_t max_response_size) const;

  HttpResponse ssl_send_then_receive(const std::string& query,
                                     std::size_t max_response_size) const;

  static const std::string HTTPS_PORT;

protected:
  const Server _server;
  const std::string _matrix_service;
  const std::string _matrix_durations_key;
  const std::string _matrix_distances_key;
  const std::string _route_service;
  const std::string _routing_args;

  HttpWrapper(const std::string& profile,
              Server server,
              std::string matrix_service,
              std::string matrix_durations_key,
              std::string matrix_distances_key,
              std::string route_service,
              std::string routing_args);

  // Response size is bounded based on the number of locations
  // involved in the query.
  HttpResponse run_query(const std::string& query,
                         std::size_t nb_locations) const;

  // Parse JSON response body and check for routing errors.
  void parse_response(rapidjson::Document& json_result,
                      const HttpResponse& response,
                      const std::vector<Location>& locs,
                      const std::string& service) const;

  // Accessors used to validate untrusted routing responses, all
  // throw a RoutingException on missing key or unexpected type.
  static const rapidjson::Value& get_member(const rapidjson::Value& value,
                                            const char* key);

  static const rapidjson::Value& get_array(const rapidjson::Value& value,
                                           const char* key);

  static const rapidjson::Value& get_first(const rapidjson::Value& value,
                                           const char* key);

  static double get_number(const rapidjson::Value& value, const char* key);

  static std::string get_string(const rapidjson::Value& value, const char* key);

  virtual std::string build_query(const std::vector<Location>& locations,
                                  const std::string& service) const = 0;

  virtual void check_response(const rapidjson::Document& json_result,
                              const std::vector<Location>& locs,
                              const std::string& service) const = 0;

  Matrices get_matrices(const std::vector<Location>& locs) const override;

  void update_sparse_matrix(const std::vector<Location>& route_locs,
                            Matrices& m,
                            std::mutex& matrix_m,
                            std::string& vehicle_geometry) const override;

  virtual bool
  duration_value_is_null(const rapidjson::Value& matrix_entry) const {
    // Same implementation for both OSRM and ORS.
    return matrix_entry.IsNull();
  }

  virtual bool
  distance_value_is_null(const rapidjson::Value& matrix_entry) const {
    // Same implementation for both OSRM and ORS.
    return matrix_entry.IsNull();
  }

  virtual UserDuration
  get_duration_value(const rapidjson::Value& matrix_entry) const {
    // Same implementation for both OSRM and ORS.
    if (!matrix_entry.IsNumber()) {
      throw RoutingException("Invalid duration value in routing response.");
    }
    return utils::round<UserDuration>(matrix_entry.GetDouble());
  }

  virtual UserDistance
  get_distance_value(const rapidjson::Value& matrix_entry) const {
    // Same implementation for both OSRM and ORS.
    if (!matrix_entry.IsNumber()) {
      throw RoutingException("Invalid distance value in routing response.");
    }
    return utils::round<UserDistance>(matrix_entry.GetDouble());
  }

  virtual const rapidjson::Value&
  get_legs(const rapidjson::Value& result) const = 0;

  virtual UserDuration get_leg_duration(const rapidjson::Value& leg) const {
    // Same implementation for both OSRM and ORS.
    return utils::round<UserDuration>(get_number(leg, "duration"));
  }

  virtual UserDistance get_leg_distance(const rapidjson::Value& leg) const {
    // Same implementation for both OSRM and ORS.
    return utils::round<UserDistance>(get_number(leg, "distance"));
  }

  virtual std::string get_geometry(rapidjson::Value& result) const {
    // Same implementation for both OSRM and ORS.
    return get_string(get_first(result, "routes"), "geometry");
  }

  void add_geometry(Route& route) const override;
};

} // namespace vroom::routing

#endif
