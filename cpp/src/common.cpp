#pragma once
#include "global.cpp"
#include "const.cpp"
#include <any>
#include <boost/cobalt.hpp>
#include <exception>
#include <fmt/include/fmt/ranges.h>
#include <fmt/include/fmt/core.h>
#include <fmt/include/fmt/std.h>
#include <fmt/include/fmt/ostream.h>

#include <boost/chrono.hpp>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

// support for time unit postfix, eg: 10ms 10min 10 day 
using namespace std::literals::chrono_literals;

// boost common prefixes
namespace cobalt = boost::cobalt;
namespace asio = boost::asio;

// custom library functions
namespace gal {
    
    // Run some code in a lambda that might protentially throw an error using a try-catch
    // silencing the error and returning a null optional to silence the error
    // A ret type must be manually supplied, lambda should not require any arguments
    template<typename Ret>
    inline std::optional<Ret> no_throw(std::function<Ret(void)> f) noexcept {
        try {
            return f();
        } catch (...) {
            return std::nullopt;
        }
    }

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
}

