#pragma once

#include <vector>
#include <string>

#include "motis/footpaths/tracks.h"

namespace motis::footpaths {
std::vector<track_info> extract_osm_tracks(std::string const& osm_file);

} // motis::footpaths