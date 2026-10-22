#include <atomic>

#ifdef ENABLE_HEAP_MONITOR
namespace alloc_monitor {
    extern std::atomic<bool> monitor_allocations;
    extern std::atomic<unsigned long> allocation_count;
    extern std::atomic<unsigned long> deallocation_count;
}
#endif