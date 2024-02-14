#include "Filters.hpp"

    

    LowPassFilter::LowPassFilter(double sampling_time, double cutoff_freq)
    :currentValue(0.0)   
    {
        double filter_TC = 1 / (2*M_PI*cutoff_freq) ; //filter time constant
        alpha = sampling_time / (sampling_time + filter_TC);
    }

    LowPassFilter::LowPassFilter(double alpha)
    :currentValue(0.0)
    ,alpha(alpha)
    {}
    LowPassFilter::~LowPassFilter(){}

    double LowPassFilter::filter(double input) {
        //y[n] = (1 - alpha) * y[n-1] + alpha * x[n]
        currentValue = alpha * input + (1 - alpha) * currentValue;
        return currentValue;
    }


    MovingAverageFilter::MovingAverageFilter(int _windowsize)
    :_windowSize(_windowsize)
    ,_current_index(0)
    {
        // _window.resize(_windowSize);
        // std::fill(_window.begin(), _window.end(), 0.0);
        _window.assign(_windowSize, 0.0);
    }

    MovingAverageFilter::~MovingAverageFilter(){}

    double MovingAverageFilter::compute_average(){
        double sum = 0.0;
        for (int i = 0; i < _windowSize; i++)
            sum += _window[i];
        return sum / _windowSize;
    }
    double MovingAverageFilter::filter(double input)
    {
    _window[_current_index] = input; // assign input to current
    //compute average of window
    double average = compute_average();

        //increment current index if not at end of window
        if (_current_index < _windowSize - 1)
            _current_index++;
        else
            _current_index = 0;

    return average;
    }
