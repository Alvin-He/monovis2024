#pragma once

#include "common.cpp"
#include "global.cpp"
#include <boost/container/set.hpp>
#include <mutex>

namespace Async {

class SignalStore {
    public:
    SignalStore() = default; 

    // create a new signal and store it, returning the signal id 
    size_t New() {
        std::unique_lock _l1 {m_storeLock};
        m_lastSignalID += 1;
        m_signalStore.insert(m_lastSignalID);
        return m_lastSignalID;
    }
    // store a signal 
    void Store(size_t value) {
        std::unique_lock _l1 {m_storeLock}; 
        m_signalStore.insert(value);
    }; 
    // check if a signal exists
    bool Check(size_t value) {
        std::unique_lock _l1 {m_storeLock}; 
        auto loc = m_signalStore.find(value);
        if (loc == m_signalStore.end()) return false; 
        else return true; 
    }
    // consume a signal (check then remove the signal from store)
    bool Consume(size_t value) {
        std::unique_lock _l1 {m_storeLock}; 
        auto loc = m_signalStore.find(value);
        if (loc == m_signalStore.end()) return false; 
        m_signalStore.erase(loc); 
        return true;
    }

    private:
    std::mutex m_storeLock; 
    size_t m_lastSignalID = 0; // protected by m_storeLock
    boost::container::set<size_t> m_signalStore; // protected by m_storeLock
};

}