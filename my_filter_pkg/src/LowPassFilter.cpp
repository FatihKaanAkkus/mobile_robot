#include <my_filter_pkg/LowPassFilter.h>

namespace my_filter_pkg
{
    LowPassFilter::LowPassFilter():
        output(0),
        cutOffFrequency(0)
    {}

    LowPassFilter::LowPassFilter(double iCutOffFrequency):
        output(0),
        cutOffFrequency(iCutOffFrequency),
        ePow(0)
    {}

    LowPassFilter::LowPassFilter(double iCutOffFrequency, double iDeltaTime):
        output(0),
        cutOffFrequency(iCutOffFrequency),
        ePow(1 - exp(-iDeltaTime * iCutOffFrequency))
    {}

    double LowPassFilter::update(double input)
    {
        output += (input-output) * ePow;
        return output;
    }

    double LowPassFilter::update(double input, double dt)
    {
        setDeltaTime(dt);
        return update(input);
    }

    double LowPassFilter::getOutput()
    {
        return output;
    }

    double LowPassFilter::getCutOffFrequency()
    {
        return cutOffFrequency;
    }

    void LowPassFilter::setCutOffFrequency(double frequency)
    {
        if(frequency >= 0)
            cutOffFrequency = frequency;
        else 
            cutOffFrequency = 0;
    }

    void LowPassFilter::setDeltaTime(double dt)
    {
        if(dt >= 0)
            ePow = 1 - exp(-dt * cutOffFrequency);
        else 
            ePow = 0;
    }
};
