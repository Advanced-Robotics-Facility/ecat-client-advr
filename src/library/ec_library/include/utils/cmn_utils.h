#ifndef CMN_UTILS_H
#define CMN_UTILS_H

#include <chrono>
#include <thread>
#include <fstream>
#include <vector>
#include <random>
#include <iostream>
#include <cstring>
#include <algorithm>

#ifndef EXTERNAL_LOG
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#endif

using sec = std::chrono::duration<double>;

//
// Small Static Buffer with no allocation
//
template <size_t SIZE>
struct CBuffT
{
public:

    size_t write(const char* b, size_t bf)
    {
        memcpy(buf+actual,b,bf);
        actual += bf;
        return 0;
    }

    size_t size() const { return actual; }
    void set_size(size_t size) { actual = size; }
    void reset() { actual = 0; }

    const char* data() const { return buf; }
    char* data() { return buf; }

private:
    size_t actual{};
    char buf[SIZE]{};

};
using CBuff = CBuffT<256u>;


/// Sleep for X Milliseconds
/// Uses OS sleep NOT Accurate!
#define MILLISLEEP(X) do { \
    std::chrono::milliseconds tim2((X));\
    std::this_thread::sleep_for(tim2);\
} while(0)

#ifndef EXTERNAL_LOG
inline void createLogger(const std::string& logName, const std::string& procName) {
    //Multithreaded console logger (with color support)
    auto console = spdlog::stdout_color_mt(logName);
    console->set_pattern("[%T.%e] ["+procName+"] [%l] %v");
}
#endif

const inline std::string make_daytime_string() {
    return "FILL IN THE BLANK";
}

template<typename TimeUnit>
inline int64_t getTsEpoch() {
    return std::chrono::duration_cast<TimeUnit>(std::chrono::system_clock::now().time_since_epoch()).count();
}

template<class T>
void printTimings(T times, const char* name, bool verbose=false)
{
    auto it = times.find(name);
    if (it == times.end() ) {
        std::cout << name <<" not recorded !" << std::endl;  
        return;
    }
    auto average = std::accumulate(begin(it->second), end(it->second), 0.0) / it->second.size();
    std::cout << name <<" takes on average: " << average << " secs [" << it->second.size() << "]" << std::endl;  

    int i=0;
    if (verbose)
        for (auto x : times[name])
            std::cout << i++ << ": " << x << " seconds" << std::endl; 
}


#endif
