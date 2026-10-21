/*
 * Diagnostic heap-allocation monitor.
 *
 * While alloc_monitor::monitor_allocations is true, the shared atomic
 * allocation_count records intercepted allocation attempts from all threads.
 *
 * Covered allocation paths:
 *   - malloc, calloc, realloc, aligned_alloc and posix_memalign through
 *     GNU linker --wrap options.
 *   - Global C++ new/new[], including aligned and nothrow overloads.
 *   - Standard containers when their allocators use these intercepted paths.
 *
 * Each C++ allocation request is counted once before allocation is attempted.
 * Internal calls to __real_* avoid double counting through the C wrappers.
 * Failed requests are counted too. realloc is counted even when it reuses
 * existing storage. Deallocations are not counted.
 *
 * The counter measures allocation attempts, not allocated bytes, live memory,
 * memory leaks, page faults or Xenomai mode switches.
 *
 * Coverage limitations:
 *   - Linker wrapping does not automatically intercept internal calls in
 *     already-built shared libraries.
 *   - Custom pools, class-specific allocators and direct memory mappings
 *     require separate instrumentation if they bypass these hooks.
 *   - Compiler-eliminated allocations produce no event.
 *   - A zero count only establishes that no intercepted requests occurred
 *     during the monitored interval and execution paths.
 *
 * Enable monitoring after initialization. Synchronize monitored threads
 * before disabling monitoring and reading the final count.
 * 
 * Diagnostic only: print each intercepted allocation attempt.
 * Output may block and cause a Cobalt mode switch, intentionally exposing
 * allocation activity during debugging. Printing does not guarantee an MSW.
 *
 * The per-thread recursion guard prevents allocations caused by this output
 * from recursively printing. Those nested allocations are not counted.
 * Concurrent threads may interleave output.
 */

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>
#include <iostream>

namespace alloc_monitor {

std::atomic<bool> monitor_allocations{false};
std::atomic<unsigned long> allocation_count{0};

static_assert(std::atomic<bool>::is_always_lock_free);
static_assert(std::atomic<unsigned long>::is_always_lock_free);

void record() noexcept
{
    // Each thread has its own flag.
    static thread_local bool printing = false;

    if (!monitor_allocations.load(std::memory_order_relaxed))
        return;

    // If cout causes another allocation, skip that nested call.
    if (printing)
        return;

    printing = true;

    const auto count =
        allocation_count.fetch_add(1, std::memory_order_relaxed) + 1;

    try {
        std::cout << "Allocation attempts: " << count << '\n';
    } catch (...) {
        // Prevent output errors from escaping this noexcept function.
    }

    printing = false;
}

} // namespace alloc_monitor

// These names are resolved by GNU ld's --wrap options.
extern "C" {
void* __real_malloc(std::size_t);
void* __real_calloc(std::size_t, std::size_t);
void* __real_realloc(void*, std::size_t);
void* __real_aligned_alloc(std::size_t, std::size_t);
int __real_posix_memalign(void**, std::size_t, std::size_t);
}

// C allocation wrappers.
extern "C" {

void* __wrap_malloc(std::size_t size)
{
    alloc_monitor::record();
    return __real_malloc(size);
}

void* __wrap_calloc(std::size_t count, std::size_t size)
{
    alloc_monitor::record();
    return __real_calloc(count, size);
}

void* __wrap_realloc(void* p, std::size_t size)
{
    alloc_monitor::record();
    return __real_realloc(p, size);
}

void* __wrap_aligned_alloc(std::size_t alignment, std::size_t size)
{
    alloc_monitor::record();
    return __real_aligned_alloc(alignment, size);
}

int __wrap_posix_memalign(void** p, std::size_t alignment,
                         std::size_t size)
{
    alloc_monitor::record();
    return __real_posix_memalign(p, alignment, size);
}

} // extern "C"

namespace {

// Count once per C++ allocation request.
// Calling __real_* prevents double counting through the C wrappers.
void* allocate_cpp(std::size_t size, std::size_t alignment)
{
    alloc_monitor::record();

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

        auto handler = std::get_new_handler();
        if (!handler)
            throw std::bad_alloc();

        handler();
    }
}

} // namespace

// Throwing C++ allocation functions.
__attribute__((noinline))
void* operator new(std::size_t n)
{
    return allocate_cpp(n, 0);
}

__attribute__((noinline))
void* operator new[](std::size_t n)
{
    return allocate_cpp(n, 0);
}

__attribute__((noinline))
void* operator new(std::size_t n, std::align_val_t alignment)
{
    return allocate_cpp(n, static_cast<std::size_t>(alignment));
}

__attribute__((noinline))
void* operator new[](std::size_t n, std::align_val_t alignment)
{
    return allocate_cpp(n, static_cast<std::size_t>(alignment));
}

// Nothrow C++ allocation functions.
void* operator new(std::size_t n, const std::nothrow_t&) noexcept
{
    try { return ::operator new(n); }
    catch (...) { return nullptr; }
}

void* operator new[](std::size_t n, const std::nothrow_t&) noexcept
{
    try { return ::operator new[](n); }
    catch (...) { return nullptr; }
}

void* operator new(std::size_t n, std::align_val_t a,
                   const std::nothrow_t&) noexcept
{
    try { return ::operator new(n, a); }
    catch (...) { return nullptr; }
}

void* operator new[](std::size_t n, std::align_val_t a,
                     const std::nothrow_t&) noexcept
{
    try { return ::operator new[](n, a); }
    catch (...) { return nullptr; }
}

// Matching deallocation functions.
#define MONITOR_DELETE(...)                              \
    __attribute__((noinline))                            \
    void operator delete(__VA_ARGS__) noexcept           \
    { std::free(p); }

#define MONITOR_DELETE_ARRAY(...)                        \
    __attribute__((noinline))                            \
    void operator delete[](__VA_ARGS__) noexcept         \
    { std::free(p); }

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