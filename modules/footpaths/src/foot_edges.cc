#include <cmath>
#include <algorithm>
#include <functional>
#include <string>

#include "utl/progress_tracker.h"
#include "utl/to_vec.h"

#include "ppr/routing/search.h"
#include "ppr/serialization/reader.h"

#include "motis/core/common/logging.h"

#include "motis/module/context/motis_parallel_for.h"

#include "motis/footpaths/foot_edges.h"
#include "motis/footpaths/thread_pool.h"
#include "boost/thread/mutex.hpp"

using namespace flatbuffers;
using namespace motis::logging;
using namespace ppr;
using namespace ppr::routing;
using namespace ppr::serialization;

namespace motis::footpaths {

struct foot_edge_info {
  nigiri::duration_t duration_{};
  std::uint16_t accessibility_{};
  double distance_{};
};

inline location to_location(geo::latlng const& pos) {
  return make_location(pos.lng_, pos.lat_);
}

inline nigiri::duration_t get_duration(route const& r) {
  return nigiri::duration_t{
      static_cast<unsigned>(std::min(
          std::round(r.duration_ / 60),
          static_cast<double>(std::numeric_limits<int16_t>::max())))
  };
}

inline uint16_t get_accessibility(route const& r) {
  return static_cast<uint16_t>(std::ceil(r.accessibility_));
}

inline ::routing::osm_type to_ppr_type(motis::footpaths::osm_type const& type) {
  switch(type) {
    case osm_type::NODE : return ::routing::osm_type::NODE;
    case osm_type::WAY : return ::routing::osm_type::EDGE;
    case osm_type::RELATION : return ::routing::osm_type::AREA;
    default : return ::routing::osm_type::UNKNOWN;
  }
}

std::vector<std::vector<foot_edge_info>> to_foot_edge_info(search_result const& result) {
  return utl::to_vec(result.routes_, [&](auto const& routes) {
    return utl::to_vec(routes, [&](auto const& r) {
      return foot_edge_info{get_duration(r), get_accessibility(r), r.distance_};
    });
  });
}

search_result route_ppr_direct(
    routing_graph const& rg, int64_t const& start_id, location const& start_loc,
    ::routing::osm_type const& start_type, std::vector<int64_t> const& destination_ids,
    std::vector<location> const& destination_locs,
    std::vector<::routing::osm_type> const& destination_types,
    search_profile const& profile, search_direction const dir) {
  return find_routes(rg, start_id, start_type, start_loc,
                     destination_ids, destination_types, destination_locs,
                     profile, dir);
}

void print_footpath_information(nigiri::timetable& tt, int n) {
  size_t station_footpaths = 0, track_footpaths = 0,  num_footpaths = 0;
  int station_count = 0, track_count = 0;
  int station_total = 0, track_total = 0;
  for (auto i = nigiri::location_idx_t{0U};
       i != tt.locations_.ids_.size(); ++i) {
    num_footpaths += tt.locations_.footpaths_out_[0][i].size();
    auto const loc_type = tt.locations_.types_[i];
    if (loc_type == nigiri::location_type::kStation) {
      station_total++;
      if (tt.locations_.footpaths_out_[n][i].empty()) {
        station_count++;
      } else {
        station_footpaths += tt.locations_.footpaths_out_[n][i].size();
      }
      continue;
    }
    if (loc_type == nigiri::location_type::kTrack) {
      track_total++;
      if (tt.locations_.footpaths_out_[n][i].empty()) {
        track_count++;
      } else {
        track_footpaths += tt.locations_.footpaths_out_[n][i].size();
      }
      continue;
    }
  }
  LOG(info) << "Total Footpaths: " << num_footpaths << " Footpaths per location: " << (double)num_footpaths/(double)tt.locations_.ids_.size();
  LOG(info) << "Station Footpaths: " << station_footpaths << " Footpaths per station: " << (double)station_footpaths/(double)station_total;
  LOG(info) << "Footpaths per station(at least 1): " << (double)station_footpaths/(double)(station_total-station_count);
  LOG(info) << "Stations that don't have an outgoing footpath " << station_count << " of " << station_total;
  LOG(info) << "Track Footpaths: " << track_footpaths << " Footpaths per track: " << (double)track_footpaths/(double)track_total;
  LOG(info) << "Footpaths per track(at least 1): " << (double)track_footpaths/(double)(track_total-track_count);
  LOG(info) << "Tracks that don't have an outgoing footpath " << track_count << " of " << track_total;
}

void print_routing_information(search_result const& result, std::string const& profile_name,
                               track_info* const track, ::routing::osm_type const& type,
                               std::vector<::routing::osm_type> const& destination_types,
                               std::vector<location> const& destination_locs,
                               std::vector<int64_t> const& destination_ids, bool const dijkstra) {
  LOG(info) << "Benutztes Profil: " << profile_name;
  int attempts = result.stats_.attempts_;
  if (attempts > 1 && dijkstra) {
    LOG(info) << "Attempts: " << attempts;
    LOG(info) << "Start extended: " << result.stats_.d_start_pts_extended_;
    LOG(info) << "Destination extended: " << result.stats_.d_destination_pts_extended_;
  }
  std::string printable_type = type == ::routing::osm_type::NODE ? "NODE" : type == ::routing::osm_type::EDGE ? "EDGE" : "AREA";
  std::vector<std::string> printable_types;
  for (auto const& d_type : destination_types) {
    printable_types.emplace_back(d_type == ::routing::osm_type::NODE ? "NODE" : d_type == ::routing::osm_type::EDGE ? "EDGE" : "AREA");
  }
  LOG(info) << "Zeit bis Startpunkt gefunden wurde:"
            << result.stats_.d_start_pts_ << " " << printable_type << " " << track->osm_id_;
  if (result.stats_.start_is_location) {
    LOG(info) << "Start ist Location";
  }
  LOG(info) << "Zeit pro Zielpunkt:"
            << result.stats_.d_destination_pts_ / destination_locs.size();
  for(auto const& d : result.stats_.destination_is_location) {
    if (d) {
      LOG(info) << "Mindestens 1 Ziel ist Location";
      break;
    }
  }
  if (dijkstra) {
    for (auto const& dijkstra_stat : result.stats_.dijkstra_statistics_) {
      LOG(info) << "Dijkstra Zeit: " << dijkstra_stat.d_total_;
      LOG(info) << "  Reached " << dijkstra_stat.goals_reached_ << " of "
                << dijkstra_stat.goals_ << " goals";
      LOG(info) << "  STARTS: " << dijkstra_stat.d_starts_;
      LOG(info) << "  GOALS: " << dijkstra_stat.d_goals_;
      LOG(info) << "  AREA_EDGES: " << dijkstra_stat.d_area_edges_;
      LOG(info) << "  LABELS TO ROUTE: " << dijkstra_stat.d_labels_to_route_;
      LOG(info) << "  SEARCH: " << dijkstra_stat.d_search_;
    }
  }
  LOG(info) << "Start: " << track->pos_;
  for(int i = 0; i < destination_locs.size(); ++i) {
    LOG(info) << "Ziel: " << destination_locs[i] << " " << destination_ids[i] << " " << printable_types[i];
  }
  LOG(info) << "Zeit für die gesamte Anfrage: " << result.stats_.d_total_ << "\n";
}

using route_fn_t = std::function<search_result(
    int64_t const& /*start_id*/,
    location const& /*start_loc*/,
    ::routing::osm_type const& /*type*/,
    std::vector<int64_t> const& /*destination_ids*/,
    std::vector<location> const& /*destination_locs*/,
    std::vector<::routing::osm_type> const& /*destination_types*/,
    std::string const& /*profile_name*/, search_profile const& /*profile*/,
    search_direction /*dir*/)>;

void compute_edges(
    foot_edge_task const& task, nigiri::timetable& tt,
    std::map<std::string, motis::ppr::profile_info> const& ppr_profiles,
    bool const /*ppr_exact*/, route_fn_t const& route_fn, boost::mutex& mutex) {
  auto const& track = task.track_;
  auto const idx = task.track_->idx_t_;
  auto const& profile_name = *task.ppr_profile_;
  search_profile profile;
  int profile_number;
  auto it = ppr_profiles.find(profile_name);
  if (it != end(ppr_profiles)) {
    profile = it->second.profile_;
    profile_number = tt.locations_.profile_.at(profile_name);
  } else {
    profile = ppr_profiles.begin()->second.profile_;
    profile_number = 0;
  }
  if (!task.tracks_in_radius_.empty()) {
    auto const destination_ids = utl::to_vec(
        task.tracks_in_radius_,
        [](auto const& s) { return s->osm_id_; });
    auto const destination_locs = utl::to_vec(
        task.tracks_in_radius_,
        [](auto const& s) {return to_location(s->pos_); });
    auto const destination_types = utl::to_vec(
        task.tracks_in_radius_,
        [](auto const& s) { return to_ppr_type(s->osm_type_); });
    auto const& type = to_ppr_type(track->osm_type_);
    auto const result = route_fn(track->osm_id_, to_location(track->pos_), type,
                                     destination_ids, destination_locs, destination_types,
                                     profile_name, profile, search_direction::FWD);
    if (result.stats_.d_total_ > 1000) {
      //print_routing_information(result, profile_name, track, type, destination_types, destination_locs, destination_ids, true);
    }
    auto const fwd_result = to_foot_edge_info(result);
    assert(fwd_result.size() == task.tracks_in_radius_.size());
    if (fwd_result.empty()) {
      return;
    }
    if (result.destinations_reached() == 0) {
      //print_routing_information(result, profile_name, track, type, destination_types, destination_locs, destination_ids, false);
      return;
    }
    for (auto station_idx = 0U; station_idx < task.tracks_in_radius_.size();
         ++station_idx) {
      auto const& fwd_routes = fwd_result[station_idx];
      auto const& target_idx = task.tracks_in_radius_[station_idx]->idx_t_;
      if (fwd_routes.empty()) {
        continue;
      }
      for(auto const& r : fwd_routes) {
        auto const from_transfer_time = tt.locations_.transfer_time_[idx];
        auto const to_transfer_time = tt.locations_.transfer_time_[target_idx];
        auto const duration =
            std::max({from_transfer_time, to_transfer_time, r.duration_});
        boost::unique_lock<boost::mutex> scoped_lock(mutex);
        tt.locations_.footpaths_out_[profile_number][idx].emplace_back(target_idx, duration);
        //tt.locations_.footpaths_out_[profile_number][target_idx].emplace_back(idx, duration);
        tt.locations_.footpaths_in_[profile_number][target_idx].emplace_back(idx, duration);
        //tt.locations_.footpaths_in_[profile_number][idx].emplace_back(target_idx, duration);
      }
    }
  }
}

void compute_foot_edges_direct( std::vector<foot_edge_task> const& tasks,
                               nigiri::timetable& tt, std::map<std::string,
                               motis::ppr::profile_info> const& ppr_profiles,
                               std::string const& ppr_graph,
                               std::size_t edge_rtree_max_size,
                               std::size_t area_rtree_max_size,
                               bool lock_rtrees, int threads, bool ppr_exact) {
  LOG(info) << "Computing footpaths (" << tasks.size() << " tasks)...";

  routing_graph rg;
  {
    scoped_timer ppr_load_timer{"Loading ppr routing graph"};
    read_routing_graph(rg, ppr_graph);
  }
  {
    scoped_timer ppr_rtree_timer{"Preparing ppr r-trees"};
    rg.prepare_for_routing(
        edge_rtree_max_size, area_rtree_max_size,
        lock_rtrees ? rtree_options::LOCK : rtree_options::PREFETCH, true);
  }

  scoped_timer timer{"Computing footpaths"};

  auto progress_tracker = utl::get_active_progress_tracker();
  progress_tracker->reset_bounds().in_high(tasks.size());
  auto const route_fn = [&](int64_t const& start_id,
                            location const& start_loc,
                            ::routing::osm_type const& type,
                            std::vector<int64_t> const& destination_ids,
                            std::vector<location> const& destination_locs,
                            std::vector<::routing::osm_type> const& destination_types,
                            std::string const& /*profile_name*/,
                            search_profile const& profile,
                            search_direction const dir) {
    return route_ppr_direct(rg, start_id, start_loc, type, destination_ids, destination_locs, destination_types, profile, dir);
  };
  for (int i = 0; i < tt.locations_.profile_.size(); ++i) {
    if (i >= tt.locations_.footpaths_out_.size()) {
      tt.locations_.footpaths_out_.emplace_back();
      tt.locations_.footpaths_in_.emplace_back();
    }
    tt.locations_.footpaths_out_[i].clear();
    tt.locations_.footpaths_out_[i][nigiri::location_idx_t{tt.locations_.src_.size() - 1}];
    tt.locations_.footpaths_in_[i].clear();
    tt.locations_.footpaths_in_[i][nigiri::location_idx_t{tt.locations_.src_.size() - 1}];
  }
  LOG(info) << "Starting thread creation.";
  thread_pool pool{static_cast<unsigned>(std::max(1, threads))};
  boost::mutex mutex;
  for (auto const& t : tasks) {
    pool.post([&, &t = t] {
      progress_tracker->increment();
      compute_edges(t, tt, ppr_profiles, ppr_exact, route_fn, mutex);
    });
  }
  LOG(info) << "Starting footpaths computation with " << std::max(1, threads) << " Threads";
  pool.join();
  /*int count = 0;
  int next_print =  tasks.size() / 20 + 1;
  for (auto const& t : tasks) {
    if (++count % next_print == 0) {
      LOG(info) << count << " footpaths computed "
                << std::chrono::duration_cast<std::chrono::duration<double, std::milli>>
                       (std::chrono::steady_clock::now() - timer.start_).count() / count
                << "ms per footpath.";
    }
    progress_tracker->increment();
    compute_edges(t, tt, ppr_profiles, ppr_exact, route_fn);
  }*/
  auto time = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>
              (std::chrono::steady_clock::now() - timer.start_).count();
  LOG(info) << tasks.size() << " footpaths computed "
            << time / tasks.size()
            << "ms per footpath.";
  time /= 1000;
  LOG(info) << "Took " << time << "s or " << time / 60 << "m or " << time / 3600 << "h";
  LOG(info) << "Footpaths precomputed.";
  for (int i = 0; i < tt.locations_.profile_.size(); ++i) {
    LOG(info) << " Profil number: " << i;
    print_footpath_information(tt, i);
  }
}


}  // namespace motis::footpaths
