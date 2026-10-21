#include <atomic>

#ifdef ENABLE_ALLOCATION_MONITOR
namespace alloc_monitor {
    extern std::atomic<bool> monitor_allocations;
    extern std::atomic<unsigned long> allocation_count;
}
#endif