#ifndef AMDAHL_SRC_UTIL_RANDOM_UTILS_H_
#define AMDAHL_SRC_UTIL_RANDOM_UTILS_H_

#include <random>
#include <numeric>

constexpr int kRandomSeed = 42;

std::mt19937& GetRandomGenerator();

int GetRandInt(int min, int max);

double GetRandDouble(double min, double max);
#endif