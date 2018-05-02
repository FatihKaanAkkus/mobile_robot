#ifndef MY_FILTER_PKG_LOW_EXP_AVERAGING_FILTER_H
#define MY_FILTER_PKG_LOW_EXP_AVERAGING_FILTER_H

#include <cmath>

namespace my_filter_pkg 
{
    class ExpAveragingFilter
    {
        public:
            ExpAveragingFilter();
            ExpAveragingFilter(double iTimeConstant);
            ExpAveragingFilter(double iTimeConstant, double iDeltaTime);

            double update(double input);
            double update(double input, double dt);

            double getOutput();
            double getTimeConstant();

            void setTimeConstant(double tau);
            void setDeltaTime(double dt);
        private:
            double output;
            double timeConstant;
            double alpha;
    };
};

#endif
