/**
 * @file main.cpp
 * @brief Entry point for the host side unit tests.
 */
#include "test_framework.h"

int main() {
  std::printf("Running ps-controller tests\n\n");
  return ::testing::runAll();
}
