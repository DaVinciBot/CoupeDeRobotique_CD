#pragma once
// Profilage léger : PROFILE_SCOPE("nom") mesure la durée d'un bloc (actif si debug).
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <map>
#include <mutex>
#include <string>

namespace utils {

inline bool& debugFlag() { static bool f = false; return f; }
inline void setDebugMode(bool v) { debugFlag() = v; }

struct TimerStats {
    double totalMs = 0.0;
    double maxMs = 0.0;
    uint64_t count = 0;
};

inline std::map<std::string, TimerStats>& timerStats() {
    static std::map<std::string, TimerStats> s;
    return s;
}
inline std::mutex& timerMutex() {
    static std::mutex m;
    return m;
}

class ScopedTimer {
public:
    explicit ScopedTimer(const char* name)
        : name_(name), start_(std::chrono::steady_clock::now()) {}
    ~ScopedTimer() {
        if (!debugFlag()) return;
        double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - start_).count();
        std::lock_guard<std::mutex> lk(timerMutex());
        auto& st = timerStats()[name_];
        st.totalMs += ms;
        st.count++;
        if (ms > st.maxMs) st.maxMs = ms;
    }
private:
    const char* name_;
    std::chrono::steady_clock::time_point start_;
};

inline void printTimerStats() {
    std::lock_guard<std::mutex> lk(timerMutex());
    for (auto& kv : timerStats()) {
        const auto& s = kv.second;
        std::fprintf(stderr, "[timer] %-28s n=%-6llu avg=%7.2f ms  max=%7.2f ms\n",
                     kv.first.c_str(), (unsigned long long)s.count,
                     s.count ? s.totalMs / s.count : 0.0, s.maxMs);
    }
}

}  // namespace utils

#define UTILS_CAT2(a, b) a##b
#define UTILS_CAT(a, b) UTILS_CAT2(a, b)
#define PROFILE_SCOPE(name) utils::ScopedTimer UTILS_CAT(_scoped_timer_, __LINE__)(name)
