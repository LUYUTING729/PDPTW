#include "util/random_utils.h"

#include <random>

std::mt19937& GetRandomGenerator() {
    static thread_local std::mt19937 generator(kRandomSeed);
    return generator;
}

int GetRandInt(int min, int max) {
    std::uniform_int_distribution<int> distribution(min, max);
    return distribution(GetRandomGenerator());
}

double GetRandDouble(double min, double max) {
    std::uniform_real_distribution<double> distribution(min, max);
    return distribution(GetRandomGenerator());
}