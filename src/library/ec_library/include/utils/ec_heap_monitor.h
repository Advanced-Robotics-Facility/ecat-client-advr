#include <atomic>

#ifdef ENABLE_HEAP_MONITOR
namespace heap_monitor {
    extern std::atomic<bool> track_heap;
    extern std::atomic<unsigned long> allocation_count;
    extern std::atomic<unsigned long> deallocation_count;
}
#endif