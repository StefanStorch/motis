#pragma once

#include <map>

#include "nigiri/timetable.h"

#include "motis/ppr/profile_info.h"

#include "motis/footpaths/foot_edge_task.h"

namespace motis::footpaths {

void compute_foot_edges_direct( std::vector<foot_edge_task> const& tasks,
    nigiri::timetable& tt, std::map<std::string, motis::ppr::profile_info> const& ppr_profiles,
    std::string const& ppr_graph, std::size_t edge_rtree_max_size,
    std::size_t area_rtree_max_size, bool lock_rtrees, int threads,
    bool ppr_exact);

void compute_foot_edges_via_module( std::vector<foot_edge_task> const& tasks,
    std::map<std::string, motis::ppr::profile_info> const& ppr_profiles,
    bool ppr_exact);

}  // namespace motis::footpaths
