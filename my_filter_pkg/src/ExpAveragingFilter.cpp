#include <my_filter_pkg/ExpAveragingFilter.h>

namespace my_filter_pkg
{
    ExpAveragingFilter::ExpAveragingFilter():
        output(0),
        timeConstant(0),
        alpha(0)
    {}

    ExpAveragingFilter::ExpAveragingFilter(double iTimeConstant):
        output(0),
        timeConstant(iTimeConstant),
        alpha(0)
    {}

    ExpAveragingFilter::ExpAveragingFilter(double iTimeConstant, double iDeltaTime):
        output(0),
        timeConstant(iTimeConstant),
        alpha(1 - exp(-iDeltaTime * iTimeConstant))
    {}

    double ExpAveragingFilter::update(double input)
    {
        output = alpha * input + (1-alpha) * output;
        return output;
    }

    double ExpAveragingFilter::update(double input, double dt)
    {
        setDeltaTime(dt);
        return update(input);
    }

    double ExpAveragingFilter::getOutput()
    {
        return output;
    }

    double ExpAveragingFilter::getTimeConstant()
    {
        return timeConstant;
    }

    void ExpAveragingFilter::setTimeConstant(double tau)
    {
        if(tau >= 0)
            timeConstant = tau;
        else 
            timeConstant = 0;
    }

    void ExpAveragingFilter::setDeltaTime(double dt)
    {
        if(dt >= 0)
            alpha = 1 - exp(-dt * timeConstant);
        else 
            alpha = 0;
    }
};
