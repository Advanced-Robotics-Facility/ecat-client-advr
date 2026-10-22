/*
 * Diagnostic heap-allocation and deallocation monitor.
 *
 * When track_heap is true, shared atomic counters record intercepted
 * allocation attempts and non-null deallocation calls from all threads.
 *
 * Covered paths:
 *   - malloc, calloc, realloc, aligned_alloc, posix_memalign and free,
 *     through GNU linker --wrap options.
 *   - Global C++ new/new[] and delete/delete[], including aligned,
 *     sized and nothrow forms where applicable.
 *   - Containers whose allocators use these intercepted functions.
 *
 * Allocation requests are counted before execution, including failed requests.
 * realloc counts as one allocation attempt, even when it reuses or releases
 * storage internally. Such internal releases are not separately counted.
 * free(nullptr) and null delete calls are not counted.
 *
 * C++ replacements call __real_* directly to avoid double counting.
 *
 * Diagnostic output may block and cause a Cobalt mode switch. This deliberately
 * disturbs real-time execution, but output does not guarantee a mode switch.
 * A per-thread recursion guard skips events caused by diagnostic output and
 * stack tracing in that thread. Concurrent threads may interleave messages.
 *
 * Counters measure calls, not bytes, live objects, memory leaks or mode switches.
 * Their difference must not be interpreted as the number of outstanding blocks.
 *
 * BACKTRACE INTERPRETATION
 *
 * backtrace() captures the current call stack; backtrace_symbols_fd() prints it.
 * The most recent frame is printed first:
 *
 *   monitor -> allocation/free hook -> application function -> caller
 *
 * Read from bottom to top to follow execution toward the monitored event.
 * The first frames normally describe the monitor itself. Later frames identify
 * the application code that requested allocation or deallocation.
 *
 * Common C++ symbols:
 *   _Znwm  : operator new(std::size_t)
 *   _ZdlPv : operator delete(void*)
 *   _ZdlPvm: operator delete(void*, std::size_t)
 *
 * BUILD WITH DEBUG INFORMATION
 *
 * Configure using one of:
 *
 *   cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug \
 *       -DENABLE_ALLOCATION_MONITOR=ON
 *
 *   cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
 *       -DENABLE_ALLOCATION_MONITOR=ON
 *
 *   cmake --build build
 *
 * These commands assume a single-configuration generator, such as Makefiles
 * or ordinary Ninja. The configuration name is "RelWithDebInfo".
 *
 * With GCC, Debug normally enables debug information with little or no
 * optimization. RelWithDebInfo normally combines optimization and debug
 * information, so functions may be inlined and frames may be optimized away.
 *
 * Debug information must be available for every relevant application/library
 * source file, not only this monitor. Keep the executable unstripped or retain
 * matching separate debug files. -rdynamic helps expose function names in the
 * printed trace, but does not provide source-line information.
 *
 * RESOLVING SOURCE LINES WITH addr2line
 *
 * Always use the exact executable/library and debug information corresponding
 * to the trace. After rebuilding, capture a new trace before resolving it.
 *
 * A frame such as:
 *
 *   executable(+0xOFFSET)[0xRUNTIME_ADDRESS]
 *
 * contains an offset from the module's load base and the runtime address:
 *
 *   load_base = runtime_address - module_offset
 *
 * For a caller frame in that same module:
 *
 *   executable_address = caller_runtime_address - load_base
 *
 * Caller frames normally contain return addresses. Subtract one byte from
 * the executable address to locate the instruction before the return point.
 * Do not apply this subtraction blindly to the current execution frame.
 *
 * Example for an already calculated executable address, 0x1893b3:
 *
 *   addr2line -e ./ec_simple_trajectory -f -C -i 0x1893b3
 *
 * Options:
 *   -e : executable or library containing the instruction
 *   -f : print function names
 *   -C : demangle C++ names
 *   -i : include available inline-call information
 *
 * Each shared library has its own load base. Resolve its frames against that
 * library, using its own address adjustment and matching debug information.
 * Non-PIE executables normally use their linked virtual addresses directly.
 *
 * RESOLVING SOURCE LINES WITH GDB
 *
 * For an executable-relative address in a PIE executable, before running:
 *
 *   gdb -batch ./ec_simple_trajectory \
 *       -ex 'info line *0x1893b3'
 *
 * Alternatively, for a frame printed as main+0x50a:
 *
 *   gdb -batch ./ec_simple_trajectory \
 *       -ex 'info line *((char *)&main + 0x509)'
 *
 * Here, 0x509 is the caller return offset 0x50a minus one.
 * Replace example addresses/offsets with values from the current trace.
 *
 * If GDB is already attached to a running process, use the runtime addresses
 * directly because its symbols have been relocated for that process.
 *
 * "??:0" or "No line number information" can indicate missing debug information,
 * an incorrect address adjustment, or an executable that does not match.
 * Optimized traces may resolve to STL headers or monitoring code because of
 * inlining; inspect all inline locations or use Debug for clearer call chains.
 *
 * Limitations:
 *   - Executable linker wrapping does not rewrite allocation calls internal
 *     to already-built shared libraries.
 *   - Custom allocators, pools and direct memory mappings may bypass hooks.
 *   - Compiler-eliminated calls produce no events.
 *   - Zero events only describes intercepted calls in the tested interval.
 *
 * Enable after initialization. Synchronize monitored threads before disabling
 * monitoring and reading final counts. Keep monitoring enabled through object
 * destruction if their deallocations must be observed. Diagnostic use only.
 */

 #include "utils/ec_heap_monitor.h"

 #ifdef ENABLE_HEAP_MONITOR
 
#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <new>
#include <execinfo.h>
#include <unistd.h>
 
namespace heap_monitor {

std::atomic<bool> track_heap{false};
std::atomic<unsigned long> allocation_count{0};
std::atomic<unsigned long> deallocation_count{0};

static_assert(std::atomic<bool>::is_always_lock_free,
            "Monitoring flag must be lock-free");

static_assert(std::atomic<unsigned long>::is_always_lock_free,
            "Monitoring counters must be lock-free");

namespace {

// Shared by allocation and deallocation recording within each thread.
thread_local bool inside_record = false;

enum class Event {
    Allocation,
    Deallocation
};
 
 void record_event(Event event) noexcept
 {
     if (inside_record ||
         !track_heap.load(std::memory_order_relaxed))
         return;
 
     // Diagnostic output must not change the caller's errno.
     const int saved_errno = errno;
 
     inside_record = true;
 
     auto& counter = (event == Event::Allocation)
         ? allocation_count
         : deallocation_count;
 
     const auto count =
         counter.fetch_add(1, std::memory_order_relaxed) + 1;
 
     try {
         std::cout
             << (event == Event::Allocation
                     ? "Allocation attempts: "
                     : "Deallocation calls: ")
             << count << '\n';
        void* frames[32];
        const int frame_count = ::backtrace(frames, 32);
        ::backtrace_symbols_fd(frames, frame_count, STDERR_FILENO);
     } catch (...) {
         // Keep output exceptions from escaping the diagnostic hook.
     }
 
     inside_record = false;
     errno = saved_errno;
 }
 
 } // namespace
 
 void record_allocation() noexcept
 {
     record_event(Event::Allocation);
 }
 
 void record_deallocation() noexcept
 {
     record_event(Event::Deallocation);
 }
 
 } // namespace heap_monitor
 
 // Resolved by GNU linker --wrap options.
 extern "C" {
 
 void* __real_malloc(std::size_t);
 void* __real_calloc(std::size_t, std::size_t);
 void* __real_realloc(void*, std::size_t);
 void* __real_aligned_alloc(std::size_t, std::size_t);
 int __real_posix_memalign(void**, std::size_t, std::size_t);
 void __real_free(void*);
 
 // C allocation wrappers.
 void* __wrap_malloc(std::size_t size)
 {
     heap_monitor::record_allocation();
     return __real_malloc(size);
 }
 
 void* __wrap_calloc(std::size_t count, std::size_t size)
 {
     heap_monitor::record_allocation();
     return __real_calloc(count, size);
 }
 
 void* __wrap_realloc(void* p, std::size_t size)
 {
     heap_monitor::record_allocation();
     return __real_realloc(p, size);
 }
 
 void* __wrap_aligned_alloc(std::size_t alignment, std::size_t size)
 {
     heap_monitor::record_allocation();
     return __real_aligned_alloc(alignment, size);
 }
 
 int __wrap_posix_memalign(void** p,
                          std::size_t alignment,
                          std::size_t size)
 {
     heap_monitor::record_allocation();
     return __real_posix_memalign(p, alignment, size);
 }
 
 // C deallocation wrapper.
 void __wrap_free(void* p)
 {
     if (p)
         heap_monitor::record_deallocation();
 
     __real_free(p);
 }
 
 } // extern "C"
 
 namespace {
 
 void* allocate_cpp(std::size_t size, std::size_t alignment)
 {
     // Count once per C++ request, before any new_handler retries.
     heap_monitor::record_allocation();
 
     if (size == 0)
         size = 1;
 
     for (;;) {
         void* p = nullptr;
 
         if (alignment == 0) {
             p = __real_malloc(size);
         } else {
             if (__real_posix_memalign(&p, alignment, size) != 0)
                 p = nullptr;
         }
 
         if (p)
             return p;
 
         const auto handler = std::get_new_handler();
         if (!handler)
             throw std::bad_alloc();
 
         handler();
     }
 }
 
 void deallocate_cpp(void* p) noexcept
 {
     if (p)
         heap_monitor::record_deallocation();
 
     // Bypass __wrap_free: this delete has already been counted.
     __real_free(p);
 }
 
 } // namespace
 
 // Throwing C++ allocation functions.
 __attribute__((noinline))
 void* operator new(std::size_t size)
 {
     return allocate_cpp(size, 0);
 }
 
 __attribute__((noinline))
 void* operator new[](std::size_t size)
 {
     return allocate_cpp(size, 0);
 }
 
 __attribute__((noinline))
 void* operator new(std::size_t size, std::align_val_t alignment)
 {
     return allocate_cpp(size, static_cast<std::size_t>(alignment));
 }
 
 __attribute__((noinline))
 void* operator new[](std::size_t size, std::align_val_t alignment)
 {
     return allocate_cpp(size, static_cast<std::size_t>(alignment));
 }
 
 // Nothrow forms delegate to the throwing forms: counted once.
 __attribute__((noinline))
 void* operator new(std::size_t size, const std::nothrow_t&) noexcept
 {
     try {
         return ::operator new(size);
     } catch (...) {
         return nullptr;
     }
 }
 
 __attribute__((noinline))
 void* operator new[](std::size_t size, const std::nothrow_t&) noexcept
 {
     try {
         return ::operator new[](size);
     } catch (...) {
         return nullptr;
     }
 }
 
 __attribute__((noinline))
 void* operator new(std::size_t size, std::align_val_t alignment,
                    const std::nothrow_t&) noexcept
 {
     try {
         return ::operator new(size, alignment);
     } catch (...) {
         return nullptr;
     }
 }
 
 __attribute__((noinline))
 void* operator new[](std::size_t size, std::align_val_t alignment,
                      const std::nothrow_t&) noexcept
 {
     try {
         return ::operator new[](size, alignment);
     } catch (...) {
         return nullptr;
     }
 }
 
 // Matching C++ deallocation functions.
 #define MONITOR_DELETE(...)                              \
     __attribute__((noinline))                            \
     void operator delete(__VA_ARGS__) noexcept           \
     { deallocate_cpp(p); }
 
 #define MONITOR_DELETE_ARRAY(...)                        \
     __attribute__((noinline))                            \
     void operator delete[](__VA_ARGS__) noexcept         \
     { deallocate_cpp(p); }
 
 MONITOR_DELETE(void* p)
 MONITOR_DELETE(void* p, std::size_t)
 MONITOR_DELETE(void* p, std::align_val_t)
 MONITOR_DELETE(void* p, std::size_t, std::align_val_t)
 MONITOR_DELETE(void* p, const std::nothrow_t&)
 MONITOR_DELETE(void* p, std::align_val_t, const std::nothrow_t&)
 
 MONITOR_DELETE_ARRAY(void* p)
 MONITOR_DELETE_ARRAY(void* p, std::size_t)
 MONITOR_DELETE_ARRAY(void* p, std::align_val_t)
 MONITOR_DELETE_ARRAY(void* p, std::size_t, std::align_val_t)
 MONITOR_DELETE_ARRAY(void* p, const std::nothrow_t&)
 MONITOR_DELETE_ARRAY(void* p, std::align_val_t, const std::nothrow_t&)
 
 #undef MONITOR_DELETE
 #undef MONITOR_DELETE_ARRAY
 
 #endif // ENABLE_HEAP_MONITOR