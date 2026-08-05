#include "low_pass_filter.h"


LowPassFilter::LowPassFilter(float alpha)
{
    alpha_ = alpha;
    output_ = 0;
    initialized_ = false;
}



float LowPassFilter::update(float input)
{
    if(!initialized_)
    {
        output_ = input;
        initialized_ = true;
    }
    else
    {
        output_ =
        alpha_ * input +
        (1.0f-alpha_) * output_;
    }

    return output_;
}



void LowPassFilter::reset(float value)
{
    output_ = value;
    initialized_ = true;
}