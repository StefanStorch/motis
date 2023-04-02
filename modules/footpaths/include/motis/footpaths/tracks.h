#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geo/latlng.h"
#include "geo/point_rtree.h"

#include "nigiri/timetable.h"

namespace motis::footpaths {
enum class osm_type : std::uint8_t { NODE, WAY, RELATION};

struct track_info {
  track_info() = default;
  track_info(int64_t id, osm_type type,  std::string name, geo::latlng pos, bool bus_stop)
      : osm_id_(id), osm_type_(type), name_(std::move(name)), pos_(pos),
        idx_t_(nigiri::location_idx_t::invalid()), bus_stop_(bus_stop) {}
  track_info(int64_t id, osm_type type,  std::string name, geo::latlng pos,
             nigiri::location_idx_t idx_t)
      : osm_id_(id), osm_type_(type), name_(std::move(name)), pos_(pos), idx_t_(idx_t) {}
  track_info(std::string name, geo::latlng pos, nigiri::location_idx_t idx_t)
      : name_(std::move(name)), pos_(pos), idx_t_(idx_t) {}

  std::string name_;
  geo::latlng pos_;
  int64_t osm_id_{-1};
  osm_type osm_type_{osm_type::NODE};
  nigiri::location_idx_t idx_t_{nigiri::location_idx_t::invalid()};
  bool bus_stop_{false};
};

struct tracks {
  explicit tracks(std::vector<track_info> tracks) : tracks_(std::move(tracks)){
    track_index_ =
        geo::make_point_rtree(tracks_, [](auto const& t) { return t.pos_; });
  }

  std::vector<std::pair<uint64_t, double>> get_tracks_in_radius(
      geo::latlng const& center, double radius) const;

  std::vector<std::pair<uint64_t, double>> get_nearest_tracks(
      geo::latlng const& center, int count) const;

  std::size_t size() const { return tracks_.size(); }

  track_info* get_track(uint64_t pos) {
    return &tracks_.at(pos);
  }

  std::vector<track_info> tracks_;
private:
  geo::point_rtree track_index_;
};

}