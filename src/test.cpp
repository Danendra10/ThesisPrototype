#include <logger/logger.hpp>
#include <chrono>
#include <cmath>
#include <array>
#include <iostream>
#include <math/trigonom.hpp>

using namespace std;
using namespace std::chrono;

logger::Logger logger_instance;
int main()
{
    auto start = steady_clock::now();

    float th = 2;
    float out;
    int index = static_cast<int>((th + 2.0) / 0.001); // Adjust index based on range and step

    for (int i = 0; i < 10000000; i++)
    {
        // out = sqrt(th);
        // out = tanh(th);
        // out = cosh_lut[index];
        out = cosh(th);
    }

    auto end = steady_clock::now();

    // Cast the duration to floating-point milliseconds
    duration<float, milli> time_diff_ms = end - start;

    // Use %f to print floating-point values
    logger_instance.Log(logger::YELLOW, "Time taken: %.3f ms result: %f", time_diff_ms.count(), out);
}
