#pragma once

#include <chrono>
#include <iostream>

class Timer
{
private:
    std::chrono::high_resolution_clock::time_point start_;
    std::chrono::high_resolution_clock::time_point end_;

public:
    Timer()
    : start_(std::chrono::high_resolution_clock::now()),
      end_(std::chrono::high_resolution_clock::now())
    {}

    void reset()
    {
        start_ = std::chrono::high_resolution_clock::now();
    }

    void stop()
    {
        end_ = std::chrono::high_resolution_clock::now();
    }

    double elapsedSeconds() const
    {
        auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(
            end_ - start_);

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
