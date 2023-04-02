#include "motis/footpaths/footpaths.h"

#include <iostream>
#include <regex>

#include "utl/verify.h"
#include "utl/to_vec.h"

#include "motis/core/common/logging.h"
#include "motis/module/event_collector.h"
#include "motis/module/context/motis_call.h"
#include "motis/module/ini_io.h"
#include "motis/module/message.h"

#include "motis/ppr/ppr.h"
#include "motis/ppr/profiles.h"

#include "motis/footpaths/osm_tracks.h"
#include "motis/footpaths/tracks.h"
#include "motis/footpaths/foot_edges.h"

#include "nigiri/timetable.h"
#include "nigiri/types.h"

using namespace motis::logging;
using namespace motis::module;
using namespace flatbuffers;

namespace motis::footpaths {

struct import_state {
  CISTA_COMPARABLE()
  named<std::string, MOTIS_NAME("osm_path")> osm_path_;
  named<cista::hash_t, MOTIS_NAME("osm_hash")> osm_hash_;
  named<size_t, MOTIS_NAME("osm_size")> osm_size_;
  named<std::string, MOTIS_NAME("ppr_graph_path")> ppr_graph_path_;
  named<cista::hash_t, MOTIS_NAME("ppr_graph_hash")> ppr_graph_hash_;
  named<size_t, MOTIS_NAME("ppr_graph_size")> ppr_graph_size_;
  named<cista::hash_t, MOTIS_NAME("ppr_profiles_hash")> ppr_profiles_hash_;
  named<int, MOTIS_NAME("max_walk_duration")> max_walk_duration_;
  named<cista::hash_t, MOTIS_NAME("schedule_hash")> schedule_hash_;
  named<bool, MOTIS_NAME("import_osm")> import_osm_;
};

inline geo::latlng to_latlng(Position const* pos) {
  return {pos->lat(), pos->lng()};
}

inline Position to_position(geo::latlng const& loc) {
  return {loc.lat_, loc.lng_};
}

inline bool match_names(std::string const& l, std::string_view const& r) {
  std::string left = std::regex_replace(
      l, std::regex("[^0-9]+"), std::string("$1"));
  std::string right = std::regex_replace(
      std::string{r}, std::regex("[^0-9]+"), std::string("$1"));
  if (right.length() == 0) {
    return false;
  }
  return left == right;
}

struct footpaths::impl {
  explicit impl(
      nigiri::timetable const& tt, /*std::string const& db_file,*/
      /*std::size_t db_max_size,*/
      std::map<std::string, ::motis::ppr::profile_info> const& ppr_profiles,
      tracks const& tracks, bool const ppr_exact)
      : tt_{tt},
        //db_{db_file, db_max_size},
        tracks_{tracks},
        ppr_exact_{ppr_exact} {}

  /*msg_ptr transfer_time_request(msg_ptr const& msg) {
    auto const req = motis_content(FootpathsTransferTimeRequest, msg);
    message_creator fbb;
    fbb.create_and_finish(
        MsgContent_FootpathsTransferTimeResponse,
        CreateFootpathsTransferTimeResponse(
            fbb,
            fbb.CreateVector(utl::to_vec(
                tracks_.get_in_radius(to_latlng(req->pos()), req->radius()),
                [&](auto const& p) { return create_tracks(fbb, p); })))
            .Union());
    return make_msg(fbb);
  }*/

private:
  nigiri::timetable const& tt_;
  std::vector<ppr::profile_info> ppr_profiles_;
  tracks const& tracks_;
  bool ppr_exact_;
};

footpaths::footpaths() : module("Footpaths", "footpaths") {
  // to add module parameters:
  // - add field to module struct (footpaths.h), e.g. int foo_;
  // - declare the parameter here, e.g.:
  //   param(foo_, "foo", "description");
  // - use command line argument "--footpaths.foo value"
  //   or in config.ini:
  //   [footpaths]
  //   foo=value
  param(max_walk_duration_, "max_walk_duration", "max walk duration (minutes)");
  param(edge_rtree_max_size_, "import.edge_rtree_max_size",
        "Maximum size for ppr edge r-tree file");
  param(area_rtree_max_size_, "import.area_rtree_max_size",
        "Maximum size for ppr area r-tree file");
  param(lock_rtrees_, "import.lock_rtrees", "Lock ppr r-trees in memory");
  param(ppr_exact_, "ppr_exact",
        "Calculate foot edges for both directions separately (otherwise assume "
        "routes in both directions are the same)");
}

footpaths::~footpaths() = default;

void footpaths::init(motis::module::registry& reg) {
  try {
    impl_ = std::make_unique<impl>(
        *get_shared_data<nigiri::timetable*>(to_res_id(global_res_id::NIGIRI_TIMETABLE)), /*db_max_size_,*/ ppr_profiles_, *tracks_, ppr_exact_);
    /*reg.register_op("/footpaths/transfer",
                    [this](auto&& m) { return impl_->transfer_time_request(m); }, {});*/

  } catch (std::exception const& e) {
    LOG(logging::warn) << "footpaths module not initialized (" << e.what() << ")";
  }
}

nigiri::osm_type to_nigiri_osm(osm_type type) {
  switch(type) {
    case(osm_type::RELATION): return nigiri::osm_type::RELATION;
    case(osm_type::NODE): return nigiri::osm_type::NODE;
    case(osm_type::WAY): return nigiri::osm_type::WAY;
  }
}

void footpaths::import(motis::module::import_dispatcher& reg) {
  std::make_shared<event_collector>(
      get_data_directory().generic_string(), "footpaths", reg,
      [this](event_collector::dependencies_map_t const& dependencies,
             event_collector::publish_fn_t const&) {
        using import::NigiriEvent;
        using import::OSMEvent;
        using import::PPREvent;

        auto& tt = *get_shared_data<nigiri::timetable*>(
            to_res_id(global_res_id::NIGIRI_TIMETABLE));
        auto const nigiri_ev =
            motis_content(NigiriEvent, dependencies.at("NIGIRI"));
        auto const osm_ev = motis_content(OSMEvent, dependencies.at("OSM"));
        auto const ppr_ev = motis_content(PPREvent, dependencies.at("PPR"));

        // log output is written to ${data}/log/${module name}.txt

        // can be used to check if data has to be recomputed
        // (see parking module for an example)
        LOG(info) << "hashes: nigiri=" << nigiri_ev->hash()
                  << ", osm=" << osm_ev->hash()
                  << ", ppr graph=" << ppr_ev->graph_hash()
                  << ", ppr profiles=" << ppr_ev->profiles_hash();

        auto const osm_file = osm_ev->path()->str();
        LOG(info) << "loading " << osm_file << "...";

        LOG(info) << "ppr profiles:";
        for (auto const& p : *ppr_ev->profiles()) {
          int pos = 0;
          if (p->name()->view() != "default") {
            pos = tt.locations_.profile_.size();
          }
          LOG(info) << p->name()->view();
          tt.locations_.profile_.insert({p->name()->view(), pos});
        }

        ::motis::ppr::read_profile_files(
            utl::to_vec(*ppr_ev->profiles(),
                        [](auto const& p) { return p->path()->str(); }),
            ppr_profiles_);
        for (auto& p : ppr_profiles_) {
          p.second.profile_.duration_limit_ = max_walk_duration_;
        }

        std::vector<track_info> stations{};
        auto extracted_osm = extract_osm_tracks(osm_file);

        for (auto i = nigiri::location_idx_t{0U};
             i != tt.locations_.ids_.size(); ++i) {
          auto const loc_type = tt.locations_.types_[i];
          if (loc_type == nigiri::location_type::kStation) {
            auto const name = tt.locations_.names_[i].view();
            stations.emplace_back(static_cast<std::string>(name), tt.locations_.coordinates_[i], i);
          }
        }
        {
          std::vector<track_info> tracks_and_stations{};
          tracks_and_stations.reserve(stations.size() + extracted_osm.size());
          tracks_and_stations.insert(tracks_and_stations.end(), extracted_osm.begin(), extracted_osm.end());
          tracks_and_stations.insert(tracks_and_stations.end(), stations.begin(), stations.end());
          tracks_ = std::make_unique<tracks>(tracks{tracks_and_stations});
        }
        std::vector<foot_edge_task> tasks{};
        scoped_timer timer("Updating OSM-Ids of tracks");
        int total = 0, found = 0;
        for (auto i = nigiri::location_idx_t{0U};
             i != tt.locations_.ids_.size(); ++i) {
          auto const loc_type = tt.locations_.types_[i];
          if (loc_type == nigiri::location_type::kTrack) {
            total++;
            bool success = false;
            int tries = 0;
            auto const coords = tt.locations_.coordinates_[i];
            while (!success && (tries++ < 10)) {
              auto tracks = tracks_->get_tracks_in_radius(coords, 40 * tries);
              for (auto& a_track : tracks) {
                auto track = tracks_->get_track(a_track.first);
                // bus stops are only searched in 120 m radius
                if (tries > 3 && track->bus_stop_) {
                  continue;
                }
                if (track->osm_id_ != -1 &&
                    track->name_ == tt.locations_.names_[i].view()) {
                  tt.locations_.osm_ids_[i] =
                      nigiri::osm_node_id_t{track->osm_id_};
                  tt.locations_.osm_types_[i] = to_nigiri_osm(track->osm_type_);
                  if (track->idx_t_ == nigiri::location_idx_t::invalid()) {
                    track->idx_t_ = i;
                  }
                  found++;
                  success = true;
                  break;
                }
              }
            }
            // if the search wasn't successful try to only match the number of the track
            if (!success) {
              tries = 0;
              while (!success && (tries++ < 10)) {
                auto tracks = tracks_->get_tracks_in_radius(coords, 40 * tries);
                for (auto& a_track : tracks) {
                  auto track = tracks_->get_track(a_track.first);
                  // bus stops are only searched in 120 m radius
                  if (tries > 3 && track->bus_stop_) {
                    continue;
                  }
                  if (track->osm_id_ != -1 &&
                      match_names(track->name_, tt.locations_.names_[i].view())) {
                    tt.locations_.osm_ids_[i] =
                        nigiri::osm_node_id_t{track->osm_id_};
                    tt.locations_.osm_types_[i] = to_nigiri_osm(track->osm_type_);
                    if (track->idx_t_ == nigiri::location_idx_t::invalid()) {
                      track->idx_t_ = i;
                    }
                    found++;
                    success = true;
                    break;
                  }
                }
              }
            }
            if (!success) {
              auto const name = tt.locations_.names_[i].view();
              tracks_->tracks_.emplace_back(static_cast<std::string>(name), tt.locations_.coordinates_[i], i);
            }
          }
        }
        int track_count = 0, station_count = 0;
        LOG(info) << found << " of " << total << " tracks found";
        /*for (auto const& p : ppr_profiles_) {
          LOG(info) << "Profile: " << p.first
                    << " querying for tracks that are "
                    << p.second.profile_.walking_speed_ * max_walk_duration_
                    << "m away";
        }*/
        int targets = 0, sources = 0;
        for (auto& track : tracks_->tracks_) {
          if (track.idx_t_ == nigiri::location_idx_t::invalid()) {
            continue;
          }
          track.osm_id_ == -1 ? station_count++ : track_count++;
          for (auto const& p : ppr_profiles_) {
            auto near_tracks = std::move(tracks_->get_tracks_in_radius(
                track.pos_,
                p.second.profile_.walking_speed_ * max_walk_duration_));
            std::vector<track_info*> tracks_in_range{};
            for (auto const& near_track : near_tracks) {
              auto to_add = tracks_->get_track(near_track.first);
              if (to_add->idx_t_ == nigiri::location_idx_t::invalid() ||
                  to_add->idx_t_ == track.idx_t_) {
                continue;
              }
              tracks_in_range.emplace_back(to_add);
              targets++;
            }
            if (!tracks_in_range.empty()) {
              sources++;
              auto task = foot_edge_task{&track, tracks_in_range, &(p.first)};
              tasks.emplace_back(std::move(task));
            }
          }
        }
        LOG(info) << "Total targets: " << targets << " Targets per source: "
                  << ((double)((double)targets) / ((double)sources)) << " Sources: " << sources;
        compute_foot_edges_direct(tasks, tt, ppr_profiles_, ppr_ev->graph_path()->str(),
                                  edge_rtree_max_size_, area_rtree_max_size_, lock_rtrees_,
                                  std::thread::hardware_concurrency(), ppr_exact_);
        LOG(info) << "Import successful";
        import_successful_ = true;
      })
      ->require("NIGIRI",
                [](msg_ptr const& msg) {
                  return msg->get()->content_type() == MsgContent_NigiriEvent;
                })
      ->require("OSM",
                [](msg_ptr const& msg) {
                  return msg->get()->content_type() == MsgContent_OSMEvent;
                })
      ->require("PPR", [](msg_ptr const& msg) {
        return msg->get()->content_type() == MsgContent_PPREvent;
      });
}

}  // namespace motis::footpaths
