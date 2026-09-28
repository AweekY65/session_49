#include <exception>
#include <iostream>

#include "test_framework.hpp"

int main() {
    int passed = 0;
    int failed = 0;
    for (const auto& tc : r2test::registry()) {
        const int before = r2test::failures();
        std::cout << "[ RUN  ] " << tc.name << "\n";
        try {
            tc.fn();
        } catch (const std::exception& e) {
            r2test::reportFailure(__FILE__, __LINE__,
                                  std::string("uncaught exception: ") +
                                      e.what());
        } catch (...) {
            r2test::reportFailure(__FILE__, __LINE__,
                                  "uncaught unknown exception");
        }
        if (r2test::failures() == before) {
            std::cout << "[  PASS ] " << tc.name << "\n";
            ++passed;
        } else {
            std::cout << "[ FAIL  ] " << tc.name << "\n";
            ++failed;
        }
    }

    std::cout << "\n========================================\n";
    std::cout << passed << " test(s) passed, " << failed
              << " test(s) failed, " << r2test::checks()
              << " assertion(s) total\n";
    return failed == 0 ? 0 : 1;
}
