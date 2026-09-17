#include "stamped_queue_pair.h"
#include <cstdint>
#include <stdexcept>

void check(bool ok) { if (!ok) throw std::runtime_error("queue pairing regression"); }

int main()
{
    const auto stamp = [](std::int64_t t) { return t; };
    std::queue<std::int64_t> odom, cloud;
    check(!pgo::alignStampedQueues(odom, cloud, stamp));
    // Lost odometry: old cloud must not be attached to the next pose.
    odom.push(200); cloud.push(100); cloud.push(200);
    check(pgo::alignStampedQueues(odom, cloud, stamp));
    check(odom.front() == 200 && cloud.front() == 200);
    odom.pop(); cloud.pop();
    // Lost cloud: old poses must be discarded too.
    odom.push(300); odom.push(400); cloud.push(400);
    check(pgo::alignStampedQueues(odom, cloud, stamp));
    check(odom.front() == 400 && cloud.front() == 400);
    odom.pop(); cloud.pop();
    // Temporarily empty stream: preserve the newer message until its peer arrives.
    odom.push(600); cloud.push(500);
    check(!pgo::alignStampedQueues(odom, cloud, stamp));
    check(odom.front() == 600 && cloud.empty());
    cloud.push(600);
    check(pgo::alignStampedQueues(odom, cloud, stamp));
    odom.pop(); cloud.pop();
    // Backlog is bounded; recover pairing after independent queue truncation.
    for (std::int64_t i = 0; i < 100; ++i) pgo::pushBounded(odom, i, 5);
    for (std::int64_t i = 90; i < 100; ++i) pgo::pushBounded(cloud, i, 3);
    check(odom.size() == 5 && cloud.size() == 3);
    check(pgo::alignStampedQueues(odom, cloud, stamp));
    check(odom.front() == 97 && cloud.front() == 97);
}
