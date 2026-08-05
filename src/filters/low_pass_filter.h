#pragma once

class LowPassFilter
{
public:

    LowPassFilter(float alpha = 0.1f);

    float update(float input);

    void reset(float value = 0);


private:

    float alpha_;
    float output_;

    bool initialized_;
};