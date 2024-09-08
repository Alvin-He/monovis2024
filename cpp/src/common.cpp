#pragma once
#include "global.cpp"
#include "const.cpp"
#include <boost/cobalt.hpp>
#include <fmt/include/fmt/ranges.h>
#include <fmt/include/fmt/core.h>
#include <fmt/include/fmt/std.h>
#include <fmt/include/fmt/ostream.h>

#include <boost/chrono.hpp>

// support for time unit postfix, eg: 10ms 10min 10 day 
using namespace std::literals::chrono_literals;

// boost common prefixes
namespace cobalt = boost::cobalt;
namespace asio = boost::asio;