//
// Created by tko on 1/28/26.
//
#pragma once
#include <vector>

namespace acoustics {
template <typename T> std::vector<T> linspace(T start, T end, std::size_t num) {
  std::vector<T> result;
  result.reserve(num);

  if (num == 0)
    return result;
  if (num == 1) {
    result.emplace_back(start);
    return result;
  }

  T step = (end - start) / (num - 1);
  for (std::size_t i = 0; i < num; ++i)
    result.emplace_back(start + step * i);

  return result;
};
template <typename T> void printVector(const std::vector<T> &vec) {
  std::cout << "[ ";
  for (const auto &element : vec) {
    std::cout << element << " ";
  }
  std::cout << "]" << std::endl;
}
} // namespace acoustics