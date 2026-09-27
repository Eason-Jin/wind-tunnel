// Runs every registered test (or those whose name contains argv[1]).

#include "Check.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>

int main(int argc, char** argv)
{
    const char* filter = argc > 1 ? argv[1] : nullptr;
    int failed = 0, passed = 0, skipped = 0;
    for (const auto& test : check::registry()) {
        if (filter && !std::strstr(test.name, filter))
            continue;
        check::state() = {};
        const auto t0 = std::chrono::steady_clock::now();
        std::string skipReason;
        try {
            test.fn();
        } catch (const check::Skip& s) {
            skipReason = s.reason;
        } catch (const std::exception& e) {
            check::fail(test.name, 0, std::string("unexpected exception: ") + e.what());
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (check::state().failures > 0) {
            ++failed;
            std::printf("FAIL  %s (%.0f ms)\n", test.name, ms);
        } else if (!skipReason.empty()) {
            ++skipped;
            std::printf("SKIP  %s: %s\n", test.name, skipReason.c_str());
        } else {
            ++passed;
            std::printf("ok    %s (%.0f ms)\n", test.name, ms);
        }
    }
    std::printf("\n%d passed, %d failed, %d skipped\n", passed, failed, skipped);
    return failed == 0 ? 0 : 1;
}
