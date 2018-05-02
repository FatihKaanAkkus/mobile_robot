#ifndef MY_FILTER_PKG_LOW_PASS_FILTER_H
#define MY_FILTER_PKG_LOW_PASS_FILTER_H

#include <cmath>

namespace my_filter_pkg 
{
    class LowPassFilter
    {
        public:
            LowPassFilter();
            LowPassFilter(double iCutOffFrequency);
            LowPassFilter(double iCutOffFrequency, double iDeltaTime);

            double update(double input);
            double update(double input, double dt);

            double getOutput();
            double getCutOffFrequency();

            void setCutOffFrequency(double frequency);
            void setDeltaTime(double dt);
            
            double output;
            double cutOffFrequency;
            double ePow;
    };
};

#endif
