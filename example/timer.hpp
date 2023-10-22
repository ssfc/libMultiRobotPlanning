#pragma once

#include <chrono>
#include <iostream>

class Timer
{
private:
    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point end_time;

public:
    Timer()
    : start_time(std::chrono::high_resolution_clock::now()),
      end_time(std::chrono::high_resolution_clock::now())
    {}

    void reset()
    {
        start_time = std::chrono::high_resolution_clock::now();
    }

    void stop()
    {
        end_time = std::chrono::high_resolution_clock::now();
    }

    double elapsedSeconds() const
    {
        auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(
            end_time - start_time);

        return timeSpan.count();
    }
};

class ScopedTimer : public Timer
{
    public:
    ScopedTimer() {}

    ~ScopedTimer()
    {
        stop();
        std::cout << "Elapsed: " << elapsedSeconds() << " s" << std::endl;
    }
};
