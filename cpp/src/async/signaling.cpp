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
    int New() {
        std::unique_lock _l1 {m_storeLock};
        m_lastSignalID += 1;
        m_signalStore.insert(m_lastSignalID);
        return m_lastSignalID;
    }
    // store a signal 
    void Store(int value) {
        std::unique_lock _l1 {m_storeLock}; 
        m_signalStore.insert(value);
    }; 
    // check if a signal exists
    bool Check(int value) {
        std::unique_lock _l1 {m_storeLock}; 
        auto loc = m_signalStore.find(value);
        if (loc == m_signalStore.end()) return false; 
        else return true; 
    }
    // consume a signal (check then remove the signal from store)
    bool Consume(int value) {
        std::unique_lock _l1 {m_storeLock}; 
        auto loc = m_signalStore.find(value);
        if (loc == m_signalStore.end()) return false; 
        m_signalStore.erase(loc); 
        return true;
    }

    private:
    std::mutex m_storeLock; 
    int m_lastSignalID = 0; // protected by m_storeLock
    boost::container::set<int> m_signalStore; // protected by m_storeLock
};

}