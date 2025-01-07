
#include "common.cpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string_view>
#include <vector>

#include <boost/beast.hpp>

namespace Network {

    // used for simulation camera stream
    cobalt::promise<std::optional<cv::Mat>> ReadFrame(std::string_view host, std::string_view port) {
        try {
        asio::ip::tcp::resolver resolver {co_await cobalt::this_coro::executor};
        auto endpoint = resolver.resolve(host, port); 

        boost::beast::tcp_stream stream {co_await cobalt::this_coro::executor};
        stream.connect(endpoint); 

        boost::beast::http::request<boost::beast::http::empty_body> req {boost::beast::http::verb::get, "/api/cam/download", 11}; 
        req.set(boost::beast::http::field::host, host); 

        boost::beast::http::write(stream, req); // write & send req

        boost::beast::flat_buffer buf; 
        boost::beast::http::response<boost::beast::http::vector_body<char>> res; 

        boost::beast::http::read(stream, buf, res); 

        if (res.result_int() != 200) co_return std::nullopt;

        co_return cv::imdecode(res.body(), cv::IMREAD_COLOR);

        } catch (...) {
            co_return std::nullopt;
        }
    }

}