#include "motis/module/context/motis_http_req.h"

#include "boost/algorithm/string/predicate.hpp"

#include "net/http/client/http_client.h"
#include "net/http/client/https_client.h"

#include "motis/core/common/logging.h"

#include "motis/module/context/get_io_service.h"

using namespace motis::logging;
using namespace net::http::client;

namespace motis::module {

constexpr auto const kMaxRedirects = 3;

struct http_request_executor
    : std::enable_shared_from_this<http_request_executor> {
  http_request_executor(boost::asio::io_service& ios, ctx::op_id id)
      : ios_{ios},
        f_{std::make_shared<ctx::future<ctx_data, net::http::client::response>>(
            std::move(id))} {}

  void make_request(net::http::client::request req) {
    l(debug, "http request {} {}",
      net::http::client::method_to_str(req.req_method), req.address);

    auto cb = [self = shared_from_this()](auto&& a,
                                          net::http::client::response&& res,
                                          boost::system::error_code ec) {
      self->on_response(a, std::move(res), ec);
    };

    if (req.use_https()) {
      make_https(ios_, req.peer())->query(req, std::move(cb));
    } else if (req.use_http()) {
      make_http(ios_, req.peer())->query(req, std::move(cb));
    } else {
      try {
        throw utl::fail("unexpected port {} (not https or http)",
                        req.address.port());
      } catch (...) {
        f_->set(std::current_exception());
      }
    }
  }

  template <typename T>
  void on_response(T, net::http::client::response&& res,
                   boost::system::error_code ec) {
    if (ec.failed()) {
      try {
        throw std::system_error{ec};
      } catch (...) {
        f_->set(std::current_exception());
      }
    } else {
      if (auto const it = res.headers.find("location");
          it != end(res.headers)) {
        redirect(it->second);
      } else {
        f_->set(std::move(res));
      }
    }
  }

  void redirect(std::string const& target) {
    if (redirect_count_ > kMaxRedirects) {
      try {
        throw utl::fail("too many redirects");
      } catch (...) {
        f_->set(std::current_exception());
      }
    }

    ++redirect_count_;
    make_request(url(target));
  }

  unsigned redirect_count_{0U};
  boost::asio::io_service& ios_;
  http_future_t f_;
};

std::shared_ptr<ctx::future<ctx_data, net::http::client::response>>
make_http_req(net::http::client::request req, boost::asio::io_context& ios,
              ctx::op_id const& id) {
  auto const http_exec = std::make_shared<http_request_executor>(ios, id);
  http_exec->make_request(std::move(req));
  return http_exec->f_;
}

std::shared_ptr<ctx::future<ctx_data, net::http::client::response>>
motis_http_req_impl(char const* src_location, net::http::client::request req) {
  if (dispatcher::direct_mode_dispatcher_ != nullptr) {
    boost::asio::io_service ios;
    auto f = make_http_req(std::move(req), ios, ctx::op_id{});
    ios.run();
    return f;
  } else {
    auto const op = ctx::current_op<ctx_data>();
    auto id = ctx::op_id(src_location);
    id.parent_index = op->id_.index;
    return make_http_req(std::move(req), get_io_service(), id);
  }
}

}  // namespace motis::module
