#pragma once
#include "global.cpp"
#include "const.cpp"
#include <boost/cobalt.hpp>
#include <exception>
#include <fmt/include/fmt/ranges.h>
#include <fmt/include/fmt/core.h>
#include <fmt/include/fmt/std.h>
#include <fmt/include/fmt/ostream.h>

#include <boost/chrono.hpp>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>

// support for time unit postfix, eg: 10ms 10min 10 day 
using namespace std::literals::chrono_literals;

// boost common prefixes
namespace cobalt = boost::cobalt;
namespace asio = boost::asio;

template<typename T> T value_or_throw(std::optional<T> opt, std::exception_ptr ptr) {
    if (!opt.has_value()) throw ptr;
    return opt.value();
}

template<typename T> T value_or_throw(std::optional<T> opt, std::string err_msg) {
    if (!opt.has_value()) throw std::runtime_error(err_msg);
    return opt.value();
}

template<typename T, typename... Args> 
requires (sizeof...(Args) > 0)
T value_or_throw(std::optional<T> opt, fmt::format_string<Args...> fmt_str, Args... fmt_args) {
    // const auto msg = 
    if (!opt.has_value()) throw std::runtime_error(fmt::format(fmt_str, fmt_args...)); 
    return opt.value();
}