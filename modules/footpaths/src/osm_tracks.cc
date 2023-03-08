#include "motis/footpaths/osm_tracks.h"

#include <osmium/io/file.hpp>
#include <osmium/handler.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/location.hpp>
#include "osmium/geom/coordinates.hpp"
#include "osmium/area/assembler.hpp"
#include "osmium/handler/node_locations_for_ways.hpp"
#include "osmium/relations/relations_manager.hpp"
#include "osmium/visitor.hpp"
#include "osmium/area/multipolygon_manager.hpp"
#include "osmium/io/pbf_input.hpp"
#include "osmium/index/map/flex_mem.hpp"
#include "boost/filesystem/fstream.hpp"
#include "boost/geometry.hpp"
#include "boost/interprocess/managed_mapped_file.hpp"
#include <iostream>

#include "motis/core/common/logging.h"

using namespace motis::logging;
namespace motis::footpaths {

osmium::geom::Coordinates calc_center(osmium::NodeRefList const& nr_list) {
  osmium::geom::Coordinates c{0.0, 0.0};

  for (auto const& nr : nr_list) {
    c.x += nr.lon();
    c.y += nr.lat();
  }

  c.x /= nr_list.size();
  c.y /= nr_list.size();

  return c;
}

class station_handler : public osmium::handler::Handler {
public:
  explicit station_handler(std::vector<track_info>& tracks, osmium::TagsFilter const& filter)
      : tracks_{tracks}, filter_{filter} {}

  void node(osmium::Node const& node) {
    auto const& tags = node.tags();
    if (osmium::tags::match_any_of(tags, filter_)) {
      add_platform(osm_type::NODE, node.id(), node.location(), tags);
    }
  }

  void way(osmium::Way const& way) {
    auto const& tags = way.tags();
    if (osmium::tags::match_any_of(tags, filter_)) {
      add_platform(osm_type::WAY, way.id(), way.envelope().bottom_left(), tags);
    }
  }

  void area(osmium::Area const& area) {
    auto const& tags = area.tags();
    if (osmium::tags::match_any_of(tags, filter_)) {
      add_platform(area.from_way() ? osm_type::WAY : osm_type::RELATION,
                   area.orig_id(),
                   calc_center(*area.cbegin<osmium::OuterRing>()), tags);
    }
  }

private:
  void add_platform(osm_type const ot, osmium::object_id_type const id,
                    osmium::geom::Coordinates const& coord,
                    osmium::TagList const& tags) {
    //TODO maybe save multiple names per stop to get better matching results
    std::string in_name;
    if (tags.has_key("name")) {
      in_name = tags.get_value_by_key("name");
    }
    if (tags.has_key("ref_name")) {
      in_name = tags.get_value_by_key("ref_name");
    }
    if (tags.has_key("local_ref")) {
      in_name = tags.get_value_by_key("local_ref");
    }
    if (tags.has_key("ref")) {
      in_name = tags.get_value_by_key("ref");
    }
    std::vector<std::string> names{};
    boost::split(names, in_name, [](char c) {
      return c == ';' || c == '/';
    });
    bool only_one = false;
    if (std::any_of(names.begin(), names.end(),[&] (std::string const& name) -> bool {
          return name.length() > 3;
        })) {
      only_one = true;
    }
    if (only_one) {
      tracks_.emplace_back(track_info{id, ot, in_name,
                                      {coord.y, coord.x}});
      return;
    }
    for (auto const& name : names) {
      tracks_.emplace_back(track_info{id, ot, name,
                                      {coord.y, coord.x}});
    }
  }

  std::vector<track_info>& tracks_;
  osmium::TagsFilter const& filter_;
};

using index_type = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
using location_handler_type = osmium::handler::NodeLocationsForWays<index_type>;

std::vector<track_info> extract_osm_tracks(std::string const& osm_file) {

  scoped_timer timer("Extracting OSM track data");

  osmium::io::File const input_file{osm_file};

  osmium::area::Assembler::config_type assembler_config;
  assembler_config.create_empty_areas = false;
  osmium::area::MultipolygonManager<osmium::area::Assembler> mp_manager{
        assembler_config};

  osmium::TagsFilter filter{false};
  filter.add_rule(true, "public_transport", "platform");
  filter.add_rule(true, "railway", "platform");
  //filter.add_rule(true, "highway", "platform");
  //filter.add_rule(true, "public_transport", "stop_area");
  // filter.add_rule(true, "public_transport", "stop_area_group");
  //filter.add_rule(true, "type", "public_transport");

  std::clog << "Extract OSM tracks: Pass 1..." << std::endl;

  {
    scoped_timer pass1_timer("Extract OSM tracks: Pass 1...");
    osmium::relations::read_relations(input_file, mp_manager);
  }

  index_type index;
  location_handler_type location_handler{index};
  std::vector<track_info> tracks;
  station_handler data_handler{tracks, filter};

  {
    scoped_timer pass2_timer("Extract OSM tracks: Pass 2...");
    std::clog << "Extract OSM tracks: Pass 2..." << std::endl;
    osmium::io::Reader reader{input_file, osmium::io::read_meta::no};
    osmium::apply(
        reader, location_handler, data_handler,
        mp_manager.handler(
            [&data_handler](const osmium::memory::Buffer& area_buffer) {
              osmium::apply(area_buffer, data_handler);
            }));

    reader.close();

    std::clog << "Extracted " << tracks.size() << " tracks from OSM"
              << std::endl;
  }

  return tracks;
}

} // motis::footpaths