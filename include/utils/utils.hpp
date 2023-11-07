#ifndef UTIlS_HPP
#define UTIlS_HPP

#include <iostream>
#include <vector>

using namespace std;

void meshgrid(const double step_size, const double total, std::vector<std::vector<double>> &X, std::vector<std::vector<double>> &Y)
{
    int size = static_cast<int>(total / step_size) + 1; // Calculate the number of points along one dimension

    // Resize the vectors to the required size
    X.resize(size);
    Y.resize(size);
    for (auto &vec : X)
        vec.resize(size);
    for (auto &vec : Y)
        vec.resize(size);

    // Fill the vectors with the meshgrid coordinates
    for (int i = 0; i < size; ++i)
    {
        for (int j = 0; j < size; ++j)
        {
            X[i][j] = j * step_size;
            Y[i][j] = i * step_size;
        }
    }
}

#endif