#pragma once 

#include "common.cpp"
#include "global.cpp"
#include <boost/asio/thread_pool.hpp>
#include <boost/asio/strand.hpp>

namespace cobalt = boost::cobalt; 
namespace asio = boost::asio; 

namespace Async {

// generalized state that is passed to all tasks 
struct State;

typedef std::function<cobalt::task<void>(std::shared_ptr<State>)> Task; 

class TaskContainer {
    public:
    typedef std::shared_ptr<TaskContainer> Ptr; 
    TaskContainer() {};
    TaskContainer(std::function<cobalt::task<void>(std::shared_ptr<State>)>& task) : m_task(std::move(task)) { }

    template<typename TaskCompatiableFunc_t>
    TaskContainer(TaskCompatiableFunc_t task) : m_task(std::move(task)) {}

    TaskContainer& operator=(std::function<cobalt::task<void>(std::shared_ptr<State>)>& task) {
        m_task = std::move(task); 
        return *this;
    }

    void SetNext(TaskContainer::Ptr next) {
        m_nextTask = next;
    }

    TaskContainer::Ptr GetNext() {
        return m_nextTask;
    }

    TaskContainer::Ptr GetRef() {
        return m_self; 
    }

    Task GetTask() {
        return m_task; 
    }

    private:
    TaskContainer::Ptr m_self {this}; 
    TaskContainer::Ptr m_nextTask = nullptr;
    std::optional<int> m_next;
    Task m_task; 
};

class TaskRunner {
    public: 
        TaskRunner(
            std::shared_ptr<asio::thread_pool> threadpool,
            std::shared_ptr<State> state
        ) : 
            m_threadpool(threadpool)
        {
            SetState(state); 
        }; 

        void SetState(std::shared_ptr<State> state) {
            this->m_state = state; 
        }; 
        std::shared_ptr<State> GetState() { return m_state; }; 

        // run a task in the threadpool
        void run(TaskContainer::Ptr task) {
            cobalt::spawn(
                asio::make_strand(m_threadpool->get_executor()), 
                task->GetTask()(m_state),
                [this, task] (std::exception_ptr ep) {
                    if (ep) {
                        try { std::rethrow_exception(ep); } 
                        catch(std::exception & e) { printf("Completed with exception %s\n", e.what()); };
                    }
                    else if (task->GetNext()) {
                        run(task->GetNext());
                    }
                }
            );
        }

    private:
        std::shared_ptr<asio::thread_pool> m_threadpool;
        std::shared_ptr<State> m_state; 
}; 
}