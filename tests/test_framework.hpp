#pragma once

// Minimal terminal-only test framework. No rendering, no dependencies.

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace r2test {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> cases;
    return cases;
}

inline int& failures() {
    static int f = 0;
    return f;
}

inline int& checks() {
    static int c = 0;
    return c;
}

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

inline void reportFailure(const std::string& file, int line,
                          const std::string& msg) {
    ++failures();
    std::cerr << "    FAIL " << file << ":" << line << " - " << msg << "\n";
}

template <typename T>
std::string toStr(const T& v) {
    return std::to_string(v);
}

inline std::string toStr(bool v) { return v ? "true" : "false"; }

}  // namespace r2test

#define R2_TEST(name)                                                     \
    static void r2_test_##name();                                         \
    static r2test::Registrar r2_reg_##name(#name, r2_test_##name);        \
    static void r2_test_##name()

#define CHECK(cond)                                                    \
    do {                                                               \
        ++r2test::checks();                                            \
        if (!(cond)) {                                                 \
            r2test::reportFailure(__FILE__, __LINE__,                  \
                                  std::string("CHECK failed: ") + #cond); \
        }                                                              \
    } while (0)

#define CHECK_MSG(cond, msg)                                              \
    do {                                                                  \
        ++r2test::checks();                                               \
        if (!(cond)) {                                                    \
            r2test::reportFailure(__FILE__, __LINE__, (msg));             \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(actual, expected, eps)                                   \
    do {                                                                    \
        ++r2test::checks();                                                 \
        double r2a_ = static_cast<double>(actual);                          \
        double r2e_ = static_cast<double>(expected);                        \
        if (std::fabs(r2a_ - r2e_) > (eps)) {                               \
            r2test::reportFailure(                                          \
                __FILE__, __LINE__,                                         \
                std::string("CHECK_NEAR failed: ") + #actual + "=" +        \
                    r2test::toStr(r2a_) + " expected " +                    \
                    r2test::toStr(r2e_) + " (eps=" #eps ")");               \
        }                                                                   \
    } while (0)

#define CHECK_NEAR_VEC2(actual, ex, ey, eps)                                \
    do {                                                                    \
        ++r2test::checks();                                                 \
        if (std::fabs((actual).x - (ex)) > (eps) ||                         \
            std::fabs((actual).y - (ey)) > (eps)) {                         \
            r2test::reportFailure(                                          \
                __FILE__, __LINE__,                                         \
                std::string("CHECK_NEAR_VEC2 failed: got (") +               \
                    r2test::toStr((actual).x) + ", " +                      \
                    r2test::toStr((actual).y) + ") expected (" +            \
                    r2test::toStr(static_cast<double>(ex)) + ", " +         \
                    r2test::toStr(static_cast<double>(ey)) + ")");          \
        }                                                                   \
    } while (0)

#define CHECK_THROWS(expr)                                                  \
    do {                                                                    \
        ++r2test::checks();                                                 \
        bool r2threw_ = false;                                              \
        try {                                                               \
            expr;                                                           \
        } catch (...) {                                                     \
            r2threw_ = true;                                                \
        }                                                                   \
        if (!r2threw_) {                                                    \
            r2test::reportFailure(__FILE__, __LINE__,                       \
                                  std::string("CHECK_THROWS failed: ") +    \
                                      #expr);                               \
        }                                                                   \
    } while (0)
