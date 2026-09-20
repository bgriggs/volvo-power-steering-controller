/**
 * @file test_framework.h
 * @brief Minimal single-header test framework.
 *
 * Deliberately dependency free: these tests must build with nothing but a C++
 * compiler so they stay runnable on any machine that can check out the repo.
 *
 * Usage:
 *   TEST(name_of_test) { EXPECT_EQ(2, 1 + 1); }
 *
 * Tests self-register at static init time and main() runs all of them.
 */
#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace testing {

struct TestFailure {
  std::string message;
};

struct TestCase {
  const char* name;
  std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> tests;
  return tests;
}

struct Registrar {
  Registrar(const char* name, std::function<void()> fn) {
    registry().push_back(TestCase{ name, fn });
  }
};

/** Renders a value for failure output. Bytes print as numbers, not characters. */
template <typename T>
inline std::string describe(const T& value) {
  std::ostringstream os;
  os << value;
  return os.str();
}
inline std::string describe(bool value) { return value ? "true" : "false"; }
inline std::string describe(uint8_t value) { return describe((unsigned)value); }
inline std::string describe(int8_t value) { return describe((int)value); }

[[noreturn]] inline void fail(const char* file, int line, const std::string& detail) {
  std::ostringstream os;
  os << file << ":" << line << "\n      " << detail;
  throw TestFailure{ os.str() };
}

/** Formats a byte buffer as space separated hex, for failure output. */
inline std::string describeBytes(const uint8_t* bytes, size_t count) {
  std::string out;
  char buf[8];
  for (size_t i = 0; i < count; i++) {
    std::snprintf(buf, sizeof(buf), " %02X", bytes[i]);
    out += buf;
  }
  return out;
}

/** Compares two byte buffers, reporting the whole buffer on mismatch. */
inline void expectBytesEq(const char* file, int line, const char* expression,
                          const uint8_t* actual, const uint8_t* expected, size_t count) {
  if (std::memcmp(actual, expected, count) == 0) return;

  std::ostringstream os;
  os << expression << "\n      expected:" << describeBytes(expected, count)
     << "\n      actual:  " << describeBytes(actual, count);
  fail(file, line, os.str());
}

/** Runs every registered test, printing one line each. Returns a process exit code. */
inline int runAll() {
  int passed = 0;
  std::vector<std::string> failures;

  for (const TestCase& test : registry()) {
    try {
      test.fn();
      std::printf("  [ ok ] %s\n", test.name);
      passed++;
    } catch (const TestFailure& failure) {
      std::printf("  [FAIL] %s\n      %s\n", test.name, failure.message.c_str());
      failures.push_back(test.name);
    } catch (const std::exception& e) {
      std::printf("  [FAIL] %s\n      unexpected exception: %s\n", test.name, e.what());
      failures.push_back(test.name);
    }
  }

  std::printf("\n%d passed, %d failed, %d total\n", passed,
              (int)failures.size(), (int)registry().size());
  for (const std::string& name : failures) {
    std::printf("  failed: %s\n", name.c_str());
  }
  return failures.empty() ? 0 : 1;
}

} // namespace testing

#define TEST(name)                                                      \
  static void name();                                                   \
  static ::testing::Registrar registrar_##name(#name, name);            \
  static void name()

#define EXPECT_TRUE(expr)                                               \
  if (!(expr)) ::testing::fail(__FILE__, __LINE__, std::string("expected true: ") + #expr)

#define EXPECT_FALSE(expr)                                              \
  if (expr) ::testing::fail(__FILE__, __LINE__, std::string("expected false: ") + #expr)

#define EXPECT_EQ(expected, actual)                                     \
  ::testing::expectEq(__FILE__, __LINE__, #actual, (expected), (actual))

#define EXPECT_NEAR(expected, actual, tolerance)                        \
  ::testing::expectNear(__FILE__, __LINE__, #actual, (expected), (actual), (tolerance))

/** Compares a buffer against an expected byte list: EXPECT_BYTES_EQ(frame, {0x01, 0x02}) */
#define EXPECT_BYTES_EQ(actual, ...)                                    \
  ::testing::expectBytesList(__FILE__, __LINE__, #actual, (actual), __VA_ARGS__)

namespace testing {

template <typename E, typename A>
inline void expectEq(const char* file, int line, const char* expression,
                     const E& expected, const A& actual) {
  if (expected == actual) return;

  std::ostringstream os;
  os << expression << "\n      expected: " << describe(expected)
     << "\n      actual:   " << describe(actual);
  fail(file, line, os.str());
}

inline void expectNear(const char* file, int line, const char* expression,
                       double expected, double actual, double tolerance) {
  double delta = expected - actual;
  if (delta < 0) delta = -delta;
  if (delta <= tolerance) return;

  std::ostringstream os;
  os << expression << "\n      expected: " << describe(expected) << " +/- "
     << describe(tolerance) << "\n      actual:   " << describe(actual);
  fail(file, line, os.str());
}

inline void expectBytesList(const char* file, int line, const char* expression,
                            const uint8_t* actual, std::initializer_list<uint8_t> expected) {
  std::vector<uint8_t> want(expected);
  expectBytesEq(file, line, expression, actual, want.data(), want.size());
}

} // namespace testing

#endif // TEST_FRAMEWORK_H
