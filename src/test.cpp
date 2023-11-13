#include <logger/logger.hpp>
#include <chrono>

using namespace std;
using namespace std::chrono;

logger::Logger logger_instance;

int main()
{
    auto start = steady_clock::now();

    int x = 0;
    for (int i = 0; i < 50000000; i++)
    {
        x += 1;
    }

    auto end = steady_clock::now();

    // Cast the duration to floating-point milliseconds
    duration<float, milli> time_diff_ms = end - start;

    // Use %f to print floating-point values
    logger_instance.Log(logger::YELLOW, "Time taken: %.3f ms", time_diff_ms.count());
}
