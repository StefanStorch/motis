#pragma once

#include "motis/ppr/profile_info.h"

#include "motis/footpaths/tracks.h"

#include "motis/module/module.h"

namespace motis::footpaths {

struct footpaths : public motis::module::module {
  footpaths();
  ~footpaths() override;

  footpaths(footpaths const&) = delete;
  footpaths& operator=(footpaths const&) = delete;

  footpaths(footpaths&&) = delete;
  footpaths& operator=(footpaths&&) = delete;

  void init(motis::module::registry&) override;
  void import(motis::module::import_dispatcher&) override;
  bool import_successful() const override { return import_successful_; }

private:
  int max_walk_duration_{600};
  std::size_t edge_rtree_max_size_{1024UL * 1024 * 1024 * 3};
  std::size_t area_rtree_max_size_{1024UL * 1024 * 1024};
  bool lock_rtrees_{false};
  bool ppr_exact_{true};

  struct impl;
  std::unique_ptr<impl> impl_;
  bool import_successful_{false};
  std::map<std::string, ::motis::ppr::profile_info> ppr_profiles_;
  std::unique_ptr<tracks> tracks_;
};

}  // namespace motis::footpaths
