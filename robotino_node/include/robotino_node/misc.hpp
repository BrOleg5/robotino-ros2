#ifndef MISC_HPP_
#define MISC_HPP_

template <typename T>
constexpr T PI = T(3.14159265358979323846264338327950288419716939937510L);

// source: https://stackoverflow.com/a/4609795
template <typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

#endif  // MISC_HPP_
