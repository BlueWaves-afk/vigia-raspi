/**
 * no_heap.cpp — ban all dynamic allocation at link time
 *
 * Any translation unit that calls new/delete will fail to compile
 * with a static_assert, making heap use a build error rather than
 * a runtime fault. Required by firmware invariant §0.
 */

#include <cstddef>

void *operator new(std::size_t) {
    static_assert(false, "Dynamic allocation is banned in Vigia firmware. "
                         "Use static or stack buffers only.");
    return nullptr;
}

void *operator new[](std::size_t) {
    static_assert(false, "Dynamic allocation is banned in Vigia firmware.");
    return nullptr;
}

void operator delete(void *) noexcept {}
void operator delete[](void *) noexcept {}
void operator delete(void *, std::size_t) noexcept {}
void operator delete[](void *, std::size_t) noexcept {}
