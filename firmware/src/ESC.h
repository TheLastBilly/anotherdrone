#pragma once

#include "Arduino.h"

#define MIN(a,b)            (((a)<(b))?(a):(b))
#define MAX(a,b)            (((a)>(b))?(a):(b))

class ESC
{
public:
    std::vector<std::pair<uint, uint>> stops;
    uint freq;
    uint counter, resolution;
    float minPulseWidthFraction, maxPulseWidthFraction;

    HardwareTimer *timer;

    ESC()
    {}

    static void callback(ESC * esc)
    {
        if(esc->stops.size() < 1)
            return;

        if(esc->counter >= esc->resolution || esc->counter == 0)
        {
            esc->counter = 0;
            for(std::pair<uint, uint> &s : esc->stops)
                digitalWrite(s.first, HIGH);
        }

        for(std::pair<uint, uint> &s : esc->stops)
            if(s.second == esc->counter)
                digitalWrite(s.first, LOW);
        
        esc->counter++;
    }

    void start()
    {
        if(this->timer == nullptr)
            return;
        
        this->counter = 0;
        this->timer->resume();
    }

    void stop()
    {
        if(this->timer == nullptr)
            return;
        
        for(std::pair<uint, uint> &s : this->stops)
            digitalWrite(s.first, LOW);
        this->timer->pause();
    }

    void begin(uint frequency, uint resolution = 200, float minWidth = 0.06f, float maxWidth = .095f)
    {
        if(this->timer != nullptr)
            delete this->timer;

        this->resolution = resolution;
        this->minPulseWidthFraction = MIN(MAX(minWidth, 0.f), 1.f);
        this->maxPulseWidthFraction = MIN(MAX(maxWidth, 0.f), 1.f);
        if(this->minPulseWidthFraction > this->maxPulseWidthFraction)
            std::swap(this->minPulseWidthFraction, this->maxPulseWidthFraction);

        TIM_TypeDef * instance = TIM4;
        this->timer = new HardwareTimer(instance);

        this->timer->setOverflow(frequency * resolution, HERTZ_FORMAT);
        this->timer->attachInterrupt(std::bind(callback, this));
        this->start();
    }

    void setSpeed(uint pin, float speed, bool abs = false)
    {
        uint counterStop = 0;
        bool found = false;

        speed = MIN(MAX(speed, 0.f), 1.0f);
        
        if(!abs)
            speed = speed*(this->maxPulseWidthFraction - this->minPulseWidthFraction) + this->minPulseWidthFraction;
        counterStop = speed * this->resolution;
        for(std::pair<uint, uint> &s : this->stops)
        {
            if(s.first != pin)
                continue;
            
            s.second = counterStop;
            found = true;
            break;
        }

        if(!found)
        {
            pinMode(pin, OUTPUT);
            this->stops.push_back(std::make_pair(pin, counterStop));
        }
    }
};