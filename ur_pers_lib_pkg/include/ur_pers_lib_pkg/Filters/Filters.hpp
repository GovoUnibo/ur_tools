#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include <iostream>
#include <vector>
#include <cmath>



    class LowPassFilter {
    public:

        LowPassFilter(double sampling_freq, double cut_off_freq);
        LowPassFilter(double alpha);
        ~LowPassFilter();
        double filter(double input);

    private:

        double currentValue;
        double alpha;

    };

    class MovingAverageFilter
    {
    public:
        MovingAverageFilter(int windowSize);
        ~MovingAverageFilter();
    double filter(double input);

    private:
        double compute_average(); 
        int _windowSize;
        std::vector<double> _window;
        int _current_index;
    };

#endif // LOWPASSFILTER_H