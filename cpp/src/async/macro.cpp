#pragma once
#include "task.cpp"
#include <boost/cobalt.hpp>

#define TASK(name) Async::TaskContainer name = [] (std::shared_ptr<Async::State> state) -> boost::cobalt::task<void> 
