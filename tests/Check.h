#pragma once

// A deliberately tiny test harness: TEST() registers a function, CHECK*
// record failures without aborting the test, SKIP() marks a test that cannot
// run here (e.g. no GPU). TestMain.cpp runs everything and returns non-zero
// on any failure, which is all CTest needs.

#include <cmath>
#include <cstdio>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace check {

struct Test {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<Test>& registry()
{
    static std::vector<Test> tests;
    return tests;
}

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

struct State {
    int failures = 0;
    bool skipped = false;
    std::string skipReason;
};

inline State& state()
{
    static State s;
    return s;
}

inline void fail(const char* file, int line, const std::string& what)
{
    ++state().failures;
    std::ostringstream ss;
    ss << "    " << file << ":" << line << ": " << what << "\n";
    std::fputs(ss.str().c_str(), stderr);
}

struct Skip {
    std::string reason;
};

} // namespace check

#define WT_CAT2(a, b) a##b
#define WT_CAT(a, b) WT_CAT2(a, b)
#define TEST(name)                                                                        \
    static void name();                                                                   \
    static const check::Registrar WT_CAT(registrar_, name)(#name, name);                  \
    static void name()

#define CHECK(cond)                                                                       \
    do {                                                                                  \
        if (!(cond))                                                                      \
            check::fail(__FILE__, __LINE__, "CHECK(" #cond ") failed");                   \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                             \
    do {                                                                                  \
        const double va_ = static_cast<double>(a), vb_ = static_cast<double>(b);          \
        if (!(std::abs(va_ - vb_) <= static_cast<double>(tol))) {                         \
            std::ostringstream ss_;                                                       \
            ss_ << "CHECK_NEAR(" #a ", " #b ", " #tol ") failed: " << va_ << " vs " << vb_; \
            check::fail(__FILE__, __LINE__, ss_.str());                                   \
        }                                                                                 \
    } while (0)

#define CHECK_THROWS(expr)                                                                \
    do {                                                                                  \
        bool threw_ = false;                                                              \
        try {                                                                             \
            (void)(expr);                                                                 \
        } catch (...) {                                                                   \
            threw_ = true;                                                                \
        }                                                                                 \
        if (!threw_)                                                                      \
            check::fail(__FILE__, __LINE__, "CHECK_THROWS(" #expr ") did not throw");     \
    } while (0)

#define SKIP(reason) throw check::Skip{reason}
