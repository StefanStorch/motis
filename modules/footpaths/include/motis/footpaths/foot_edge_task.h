#pragma once

#include <string>
#include <utility>
#include <vector>

#include "motis/footpaths/tracks.h"

namespace motis::footpaths {

struct foot_edge_task {
  friend std::ostream& operator<<(std::ostream& os, foot_edge_task const& task) {
    os << "Origin-ID: " << task.track_->osm_id_
       << " Origin Name: " << task.track_->name_ << "\n";
    for (auto const& track : task.tracks_in_radius_) {
      os << "Target-ID: " << track->osm_id_
         << " Target Name: " << track->name_ << "\n";
    }
    return os;
  }

  track_info* const track_{};
  std::vector<track_info*> tracks_in_radius_;
  std::string const* ppr_profile_;
};

}  // namespace motis::footpaths
