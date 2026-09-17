#pragma once

#include <cstddef>
#include <queue>

namespace pgo {
// FAST-LIO stamps the body cloud and its pose with the same scan-end time.
// A missing message must discard its unmatched partner, never shift all later
// scan/pose pairs. Caller holds the shared input-buffer mutex.
template<class A, class B, class Stamp>
bool alignStampedQueues(std::queue<A>& a, std::queue<B>& b, Stamp stamp)
{
    while (!a.empty() && !b.empty()) {
        const auto ta = stamp(a.front());
        const auto tb = stamp(b.front());
        if (ta == tb) return true;
        if (ta < tb) a.pop();
        else b.pop();
    }
    return false;
}

template<class T>
void pushBounded(std::queue<T>& queue, const T& value, std::size_t limit)
{
    if (limit == 0) return;
    while (queue.size() >= limit) queue.pop();
    queue.push(value);
}
}  // namespace pgo
