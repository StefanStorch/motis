#include "motis/footpaths/tracks.h"

#include "utl/to_vec.h"

namespace motis::footpaths {

std::vector<std::pair<uint64_t, double>> tracks::get_tracks_in_radius(
    geo::latlng const& center, double radius) const {
  return utl::to_vec(
      track_index_.in_radius_with_distance(center, radius), [&](auto const& s) {
        auto pos = std::get<1>(s);
        return std::pair(pos, std::get<0>(s));
      });
}

std::vector<std::pair<uint64_t, double>> tracks::get_nearest_tracks(
    geo::latlng const& center, int count) const {
  return utl::to_vec(
      track_index_.nearest(center, count), [&](auto const& s) {
        auto pos = std::get<1>(s);
        return std::pair(pos, std::get<0>(s));
      });
}

}  // namespace motis::parking