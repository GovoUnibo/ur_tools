#pragma once

#include <chrono>
#include <iostream>
#include <boost/thread.hpp>
#include <ros/ros.h>    

// NON FUNZIONA

//voglio un thread che mi stampa ogni tot secondi un messaggio

namespace RosTimerUtil{
    
    class Timer {
    public:
        Timer(double duration_seconds);
        ~Timer();
        void start();
        bool isExpired() const;
        void setDuration(double duration_seconds);
        void reset();
        void stop();

    private:
        void startThread();
        double duration;
        bool timeout;
        std::chrono::time_point<std::chrono::steady_clock> start_time, current_time, elapsed_time;
        boost::thread timer_thread;


    };


} // namespace RosTimerUtil