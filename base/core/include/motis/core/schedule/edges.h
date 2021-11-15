#pragma once

#include <cassert>
#include <algorithm>

#include "motis/vector.h"

#include "motis/core/common/constants.h"
#include "motis/core/schedule/bitfield.h"
#include "motis/core/schedule/connection.h"
#include "motis/core/schedule/time.h"

namespace motis {

struct node;

struct edge_cost {
  edge_cost()
      : connection_(nullptr),
        time_(INVALID_DURATION),
        price_(0),
        transfer_(false),
        accessibility_(0) {}

  edge_cost(duration_t time, light_connection const* c)
      : connection_(c),
        time_(time),
        price_(0),
        transfer_(false),
        accessibility_(0) {}

  explicit edge_cost(duration_t time, bool transfer = false, uint16_t price = 0,
                     uint16_t accessibility = 0)
      : connection_(nullptr),
        time_(time),
        price_(price),
        transfer_(transfer),
        accessibility_(accessibility) {}

  bool is_valid() const { return time_ != INVALID_DURATION; }

  light_connection const* connection_;
  duration_t time_;
  uint16_t price_;
  bool transfer_;
  uint16_t accessibility_;
};

const edge_cost NO_EDGE = edge_cost();

enum class search_dir { FWD, BWD };

class edge {
public:
  enum type {
    INVALID_EDGE,
    ROUTE_EDGE,
    FWD_ROUTE_EDGE,
    BWD_ROUTE_EDGE,
    FOOT_EDGE,
    AFTER_TRAIN_FWD_EDGE,
    AFTER_TRAIN_BWD_EDGE,
    MUMO_EDGE,
    THROUGH_EDGE,
    ENTER_EDGE,
    EXIT_EDGE,
    FWD_EDGE,
    BWD_EDGE
  };

  edge() : from_{nullptr}, to_{nullptr} {}

  /** route edge constructor. */
  edge(node* const from, node* const to,
       mcd::vector<light_connection> const& connections,
       size_t const route_traffic_days)
      : from_{from}, to_{to} {
    m_.type_ = ROUTE_EDGE;
    if (!connections.empty()) {
      m_.route_edge_.bitfield_idx_ = route_traffic_days;
      m_.route_edge_.init_empty();
      m_.route_edge_.conns_.set(begin(connections), end(connections));
      std::sort(begin(m_.route_edge_.conns_), end(m_.route_edge_.conns_),
                d_time_cmp{});
    }
  }

  /** oneway route edge constructor. */
  edge(node* const from, node* const to,
       std::vector<light_connection> const& connections,
       size_t const route_traffic_days, search_dir const dir)
      : from_{from}, to_{to} {
    if (dir == search_dir::FWD) {
      m_.type_ = FWD_ROUTE_EDGE;
      std::sort(begin(m_.route_edge_.conns_), std::end(m_.route_edge_.conns_),
                d_time_cmp{});
    } else if (dir == search_dir::BWD) {
      m_.type_ = BWD_ROUTE_EDGE;
      std::sort(begin(m_.route_edge_.conns_), std::end(m_.route_edge_.conns_),
                a_time_cmp{});
    }

    if (!connections.empty()) {
      m_.route_edge_.init_empty();
      m_.route_edge_.conns_.set(begin(connections), std::end(connections));
      m_.route_edge_.bitfield_idx_ = route_traffic_days;
    }
  }

  /** foot edge constructor. */
  edge(node* from, node* to, uint8_t type, uint16_t time_cost, uint16_t price,
       bool transfer, int mumo_id = 0, uint16_t accessibility = 0)
      : from_(from), to_(to) {
    m_.type_ = type;
    m_.foot_edge_.time_cost_ = time_cost;
    m_.foot_edge_.price_ = price;
    m_.foot_edge_.transfer_ = transfer;
    m_.foot_edge_.mumo_id_ = mumo_id;
    m_.foot_edge_.accessibility_ = accessibility;

    assert(m_.type_ != ROUTE_EDGE);
  }

  template <search_dir Dir = search_dir::FWD>
  edge_cost get_edge_cost(time const start_time,
                          light_connection const* last_con) const {
    switch (m_.type_) {
      case ROUTE_EDGE: return get_route_edge_cost<Dir>(start_time);

      case ENTER_EDGE:
        if (Dir == search_dir::FWD) {
          return get_foot_edge_no_cost();
        } else {
          return last_con == nullptr ? NO_EDGE : get_foot_edge_cost();
        }

      case EXIT_EDGE:
        if (Dir == search_dir::FWD) {
          return last_con == nullptr ? NO_EDGE : get_foot_edge_cost();
        } else {
          return get_foot_edge_no_cost();
        }

      case AFTER_TRAIN_FWD_EDGE:
        if (Dir == search_dir::FWD) {
          return last_con == nullptr ? NO_EDGE : get_foot_edge_cost();
        } else {
          return NO_EDGE;
        }

      case AFTER_TRAIN_BWD_EDGE:
        if (Dir == search_dir::BWD) {
          return last_con == nullptr ? NO_EDGE : get_foot_edge_cost();
        } else {
          return NO_EDGE;
        }

      case FWD_EDGE:
        if (Dir == search_dir::FWD) {
          return get_foot_edge_cost();
        } else {
          return NO_EDGE;
        }

      case BWD_EDGE:
        if (Dir == search_dir::BWD) {
          return get_foot_edge_cost();
        } else {
          return NO_EDGE;
        }

      case MUMO_EDGE:
      case FOOT_EDGE: return get_foot_edge_cost();

      case THROUGH_EDGE:
        return last_con == nullptr ? NO_EDGE : edge_cost(0, false, 0);

      default: return NO_EDGE;
    }
  }

  bool operates_on_day(day_idx_t const day) {
    assert(type() == ROUTE_EDGE || type() == BWD_ROUTE_EDGE);
    return m_.route_edge_.traffic_days_->test(day);
  }

  inline edge_cost get_foot_edge_cost() const {
    return edge_cost(m_.foot_edge_.time_cost_, m_.foot_edge_.transfer_,
                     m_.foot_edge_.price_, m_.foot_edge_.accessibility_);
  }

  static inline edge_cost get_foot_edge_no_cost() {
    return edge_cost(0, false, 0, 0);
  }

  edge_cost get_minimum_cost() const {
    if (m_.type_ == INVALID_EDGE) {
      return NO_EDGE;
    } else if (m_.type_ == ROUTE_EDGE) {
      if (m_.route_edge_.conns_.empty()) {
        return NO_EDGE;
      } else {
        return edge_cost(
            std::min_element(
                begin(m_.route_edge_.conns_), std::end(m_.route_edge_.conns_),
                [](light_connection const& c1, light_connection const& c2) {
                  return c1.travel_time() < c2.travel_time();
                })
                ->travel_time(),
            false, begin(m_.route_edge_.conns_)->full_con_->price_);
      }
    } else if (m_.type_ == FOOT_EDGE || m_.type_ == AFTER_TRAIN_FWD_EDGE ||
               m_.type_ == AFTER_TRAIN_BWD_EDGE || m_.type_ == ENTER_EDGE ||
               m_.type_ == EXIT_EDGE || m_.type_ == BWD_EDGE) {
      return edge_cost(0, m_.foot_edge_.transfer_);
    } else if (m_.type_ == MUMO_EDGE) {
      return edge_cost(0, false, m_.foot_edge_.price_,
                       m_.foot_edge_.accessibility_);
    } else {
      return edge_cost(0);
    }
  }

  inline light_connection const* get_next_valid_lcon(light_connection const* lc,
                                                     unsigned skip = 0) const {
    assert(type() == ROUTE_EDGE);
    assert(lc);

    auto it = lc;
    while (it != end(m_.route_edge_.conns_)) {
      if (skip == 0 && (it->valid_ != 0U)) {
        return it;
      }
      ++it;
      if (skip != 0) {
        --skip;
      }
    }
    return nullptr;
  }

  inline light_connection const* get_prev_valid_lcon(light_connection const* lc,
                                                     unsigned skip = 0) const {
    assert(type() == ROUTE_EDGE);
    assert(lc);

    auto it = std::reverse_iterator<light_connection const*>(lc);
    --it;
    while (it != m_.route_edge_.conns_.rend()) {
      if (skip == 0 && (it->valid_ != 0U)) {
        return &*it;
      }
      ++it;
      if (skip != 0) {
        --skip;
      }
    }
    return nullptr;
  }

  template <search_dir Dir = search_dir::FWD>
  std::pair<light_connection const*, day_idx_t> get_connection(
      time const start_time) const {
    assert(type() == ROUTE_EDGE || type() == FWD_ROUTE_EDGE ||
           type() == BWD_ROUTE_EDGE);
    assert(start_time >= 0);

    if (m_.route_edge_.conns_.empty()) {
      return {nullptr, 0};
    }

    // TODO
    // assume traffic in BWD mode as bitfields were built assuming fwd bitfields
    /*
    bool has_traffic = true;
    if constexpr (Dir == search_dir::FWD) {
      has_traffic = false;
      if (m_.route_edge_.traffic_days_ != nullptr) {
        auto const last_day = (start_time + MAX_TRAVEL_TIME_MINUTES).day();
        for (int day_idx = start_time.day(); day_idx <= last_day; ++day_idx) {
          has_traffic |= m_.route_edge_.traffic_days_->test(day_idx);
        }
        if (!has_traffic) {
          return {nullptr, 0};
        }
      }
    }
    */

    if (Dir == search_dir::FWD) {
      auto it = std::lower_bound(
          begin(m_.route_edge_.conns_), std::end(m_.route_edge_.conns_),
          light_connection{start_time.mam(), 0U}, d_time_cmp{});

      auto const abort_time = start_time + MAX_TRAVEL_TIME_MINUTES;
      auto day = static_cast<uint16_t>(start_time.day());

      while (true) {
        if (day >= MAX_DAYS) {
          return {nullptr, 0};
        }

        if (it == end(m_.route_edge_.conns_)) {
          it = begin(m_.route_edge_.conns_);
          day += 1;
          continue;
        }

        if (it->event_time(event_type::DEP, day) > abort_time) {
          return {nullptr, 0};
        }

        if (it->traffic_days_->test(day) && it->valid_) {
          return {get_next_valid_lcon(&*it), day};
        } else {
          ++it;
        }
      }
    } else {
      auto it = std::lower_bound(
          std::rbegin(m_.route_edge_.conns_), std::rend(m_.route_edge_.conns_),
          light_connection{0U, start_time.mam()}, a_time_cmp{});

      auto const abort_time = start_time - MAX_TRAVEL_TIME_MINUTES;
      auto day = static_cast<uint16_t>(start_time.day());

      while (true) {
        if (day >= MAX_DAYS) {
          // will execute as day becomes uint16 max value
          return {nullptr, 0};
        }

        if (it == std::rend(m_.route_edge_.conns_)) {
          it = std::rbegin(m_.route_edge_.conns_);
          day -= 1;
          continue;
        }

        if (it->event_time(event_type::ARR, day) < abort_time) {
          return {nullptr, 0};
        }

        if (it->traffic_days_->test(day) && it->valid_) {
          return {get_prev_valid_lcon(&*it), day};
        } else {
          ++it;
        }
      }
    }
  }

  template <search_dir Dir = search_dir::FWD>
  edge_cost get_route_edge_cost(time const start_time) const {
    assert(type() == ROUTE_EDGE || type() == FWD_ROUTE_EDGE ||
           type() == BWD_ROUTE_EDGE);

    if (((type() == FWD_ROUTE_EDGE && Dir != search_dir::FWD) ||
         (type() == BWD_ROUTE_EDGE && Dir != search_dir::BWD))) {
      return NO_EDGE;
    }

    auto const [c, day] = get_connection<Dir>(start_time);
    return (c == nullptr)
               ? NO_EDGE
               : edge_cost(
                     (Dir == search_dir::FWD)
                         ? c->event_time(event_type::ARR, day) - start_time
                         : start_time - c->event_time(event_type::DEP, day),
                     c, day);
  }

  template <search_dir Dir = search_dir::FWD>
  inline node const* get_destination() const {
    return (Dir == search_dir::FWD) ? to_ : from_;
  }

  template <search_dir Dir = search_dir::FWD>
  inline node const* get_source() const {
    return (Dir == search_dir::FWD) ? from_ : to_;
  }

  inline node const* get_destination(search_dir dir = search_dir::FWD) const {
    return (dir == search_dir::FWD) ? to_ : from_;
  }

  inline node const* get_source(search_dir dir = search_dir::FWD) const {
    return (dir == search_dir::FWD) ? from_ : to_;
  }

  inline bool valid() const { return type() != INVALID_EDGE; }

  inline uint8_t type() const { return m_.type_; }

  inline char const* type_str() const {
    switch (type()) {
      case ROUTE_EDGE: return "ROUTE_EDGE";
      case FOOT_EDGE: return "FOOT_EDGE";
      case AFTER_TRAIN_FWD_EDGE: return "AFTER_TRAIN_FWD_EDGE";
      case AFTER_TRAIN_BWD_EDGE: return "AFTER_TRAIN_BWD_EDGE";
      case MUMO_EDGE: return "MUMO_EDGE";
      case THROUGH_EDGE: return "THROUGH_EDGE";
      case ENTER_EDGE: return "ENTER_EDGE";
      case EXIT_EDGE: return "EXIT_EDGE";
      case FWD_EDGE: return "FWD_EDGE";
      case BWD_EDGE: return "BWD_EDGE";
      default: return "INVALID";
    }
  }

  int get_mumo_id() const {
    assert(type() == MUMO_EDGE);
    return m_.foot_edge_.mumo_id_;
  }

  inline bool empty() const {
    return type() != ROUTE_EDGE || m_.route_edge_.conns_.empty();
  }

  ptr<node> from_;
  ptr<node> to_;

  union edge_details {
    edge_details() {  // NOLINT
      std::memset(static_cast<void*>(this), 0, sizeof(*this));  // NOLINT
    }

    edge_details(edge_details&& other) noexcept {  // NOLINT
      type_ = other.type_;
      if (type_ == ROUTE_EDGE) {
        route_edge_.init_empty();
        route_edge_ = std::move(other.route_edge_);
      } else {
        foot_edge_ = other.foot_edge_;
      }
    }

    edge_details(edge_details const& other) {  // NOLINT
      type_ = other.type_;
      if (type_ == ROUTE_EDGE) {
        route_edge_.init_empty();
        route_edge_ = other.route_edge_;
      } else {
        foot_edge_.init_empty();
        foot_edge_ = other.foot_edge_;
      }
    }

    edge_details& operator=(edge_details&& other) noexcept {
      type_ = other.type_;
      if (type_ == ROUTE_EDGE) {
        route_edge_.init_empty();
        route_edge_ = std::move(other.route_edge_);
      } else {
        foot_edge_.init_empty();
        foot_edge_ = other.foot_edge_;
      }

      return *this;
    }

    edge_details& operator=(edge_details const& other) {
      if (this == &other) {
        return *this;
      }

      type_ = other.type_;
      if (type_ == ROUTE_EDGE) {
        route_edge_.init_empty();
        route_edge_ = other.route_edge_;
      } else {
        foot_edge_.init_empty();
        foot_edge_ = other.foot_edge_;
      }

      return *this;
    }

    ~edge_details() {
      if (type_ == ROUTE_EDGE) {
        using Type = decltype(route_edge_.conns_);
        route_edge_.conns_.~Type();
      }
    }

    // placeholder
    uint8_t type_;

    // TYPE = ROUTE_EDGE
    struct re {
      uint8_t type_padding_;
      mcd::vector<light_connection> conns_;
      union {
        size_t bitfield_idx_;
        bitfield const* traffic_days_;
      };

      void init_empty() { new (&conns_) mcd::vector<light_connection>(); }
    } route_edge_;

    // TYPE = FOOT_EDGE & CO
    struct fe {
      uint8_t type_padding_;

      // edge weight
      uint16_t time_cost_;
      uint16_t price_;
      bool transfer_;

      // id for mumo edge
      int32_t mumo_id_;

      uint16_t accessibility_;

      void init_empty() {
        time_cost_ = 0;
        price_ = 0;
        transfer_ = false;
        mumo_id_ = -1;
        accessibility_ = 0;
      }
    } foot_edge_;
  } m_;
};

/* convenience helper functions to generate the right edge type */

inline edge make_route_edge(node* from, node* to,
                            mcd::vector<light_connection> const& connections,
                            size_t const bitfield_idx) {
  return {from, to, connections, bitfield_idx};
}

inline edge make_foot_edge(node* from, node* to, uint16_t time_cost = 0,
                           bool transfer = false) {
  return {from, to, edge::FOOT_EDGE, time_cost, 0, transfer};
}

inline edge make_after_train_fwd_edge(node* from, node* to,
                                      uint16_t time_cost = 0,
                                      bool transfer = false) {
  return {from, to, edge::AFTER_TRAIN_FWD_EDGE, time_cost, 0, transfer};
}

inline edge make_after_train_bwd_edge(node* from, node* to,
                                      uint16_t time_cost = 0,
                                      bool transfer = false) {
  return {from, to, edge::AFTER_TRAIN_BWD_EDGE, time_cost, 0, transfer};
}

inline edge make_mumo_edge(node* from, node* to, uint16_t time_cost = 0,
                           uint16_t price = 0, uint16_t accessibility = 0,
                           int mumo_id = 0) {
  return {from,  to,    edge::MUMO_EDGE, time_cost,
          price, false, mumo_id,         accessibility};
}

inline edge make_invalid_edge(node* from, node* to) {
  return {from, to, edge::INVALID_EDGE, 0, 0, false, 0};
}

inline edge make_through_edge(node* from, node* to) {
  return {from, to, edge::THROUGH_EDGE, 0, 0, false, 0};
}

inline edge make_enter_edge(node* from, node* to, uint16_t time_cost = 0,
                            bool transfer = false) {
  return {from, to, edge::ENTER_EDGE, time_cost, 0, transfer};
}

inline edge make_exit_edge(node* from, node* to, uint16_t time_cost = 0,
                           bool transfer = false) {
  return {from, to, edge::EXIT_EDGE, time_cost, 0, transfer};
}

inline edge make_fwd_edge(node* from, node* to, uint16_t time_cost = 0,
                          bool transfer = false) {
  return {from, to, edge::FWD_EDGE, time_cost, 0, transfer};
}

inline edge make_bwd_edge(node* from, node* to, uint16_t time_cost = 0,
                          bool transfer = false) {
  return {from, to, edge::BWD_EDGE, time_cost, 0, transfer};
}

}  // namespace motis
