#pragma once 

#include <shared_mutex>
#include <mutex>

namespace Async {

// Indicator that this resource is atomic and can be access/called in threads with out locking
template<typename T>
using AtomicRes = std::shared_ptr<T>;


// Thread safe wrapper around any object with shared locking for concurrent read synced write
template <typename T>
class SharedRes : public std::shared_ptr<T>, public std::shared_mutex {
    public: 
    SharedRes() : std::shared_ptr<T>(), std::shared_mutex() {}

    // Create a SharedRes from an existing shared pointer
    SharedRes(std::shared_ptr<T> ptr) : std::shared_ptr<T>(ptr), std::shared_mutex() {}

    // Create a SharedRes from a existing pointer, this pointer must exist for the life time of this SharedRes! 
    explicit SharedRes(T* ptr) : std::shared_ptr<T>(ptr), std::shared_mutex() {}


    SharedRes(const SharedRes& other) : std::shared_ptr<T>(other), std::shared_mutex() {}; 
    SharedRes& operator=(const SharedRes&) = delete;
}; 

// Thread safe wrapper around any object with shared locking for concurrent read synced write
template <typename T>
class UniqueRes : public std::unique_ptr<T>, public std::mutex {
    public: 
    UniqueRes() : std::unique_ptr<T>(), std::mutex() {}

    // Create a SharedRes from an existing shared pointer
    UniqueRes(std::unique_ptr<T> ptr) : std::unique_ptr<T>(ptr), std::shared_mutex() {}

    // Create a UniqueRes from a existing pointer, this pointer must exist for the life time of this UniqueRes! 
    explicit UniqueRes(T* ptr) : std::unique_ptr<T>(ptr), std::mutex() {}


    UniqueRes(const UniqueRes& other) : std::unique_ptr<T>(other), std::mutex() {}; 
    UniqueRes& operator=(const UniqueRes&) = delete;
}; 
};