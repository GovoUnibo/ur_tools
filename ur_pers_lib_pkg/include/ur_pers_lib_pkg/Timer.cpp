#include "Timer.hpp"

using namespace std;

namespace RosTimerUtil{


    Timer::Timer(double duration_seconds) : duration(duration_seconds), timeout(false) {}

    Timer::~Timer() {
        stop();
    }
    
    void Timer::start() {
        if (timer_thread.joinable()) 
            timer_thread.join();
        
        this->timer_thread = boost::thread(&Timer::startThread, this);
    }

    void Timer::startThread() {
        timeout = false;
        auto start_time = std::chrono::steady_clock::now();
        int rate_hz = 1;
        ros::Rate rate(rate_hz);
        
        while (!timeout) {
            

            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

            std::cout << "\033[1;32m" << "Time elapsed: " << elapsed_time << "\033[0m" << std::endl;

            if (elapsed_time >= duration) {
                timeout = true;
                std::cout << "\033[1;31m" << "Timeout of " << duration << " seconds expired" << "\033[0m" << std::endl;
            }

            rate.sleep();            
        }
    }

    bool Timer::isExpired() const {
        return timeout;
    }

    void Timer::setDuration(double duration_seconds) {
        reset();
        duration = duration_seconds;
    }

    void Timer::reset() {
        timeout = false;
    }

    void Timer::stop() {
        timeout = true;
        if (timer_thread.joinable()) {
            timer_thread.join();
        }
    }



} // namespace RosTimerUtil