#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "fr_client_library/spsc_queue.hpp"

using fr_client_library::SpscQueue;

TEST(SpscQueue, FifoOrderFullAndEmpty)
{
  SpscQueue<int, 4> q;
  int v = -1;
  EXPECT_FALSE(q.pop(v));
  for (int i = 1; i <= 4; ++i) {EXPECT_TRUE(q.push(i));}
  EXPECT_FALSE(q.push(5));          // full: the element is not stored
  EXPECT_EQ(q.size(), 4u);
  for (int i = 1; i <= 4; ++i) {
    ASSERT_TRUE(q.pop(v));
    EXPECT_EQ(v, i);
  }
  EXPECT_FALSE(q.pop(v));
  EXPECT_EQ(q.size(), 0u);
}

TEST(SpscQueue, WrapsAround)
{
  SpscQueue<int, 4> q;
  int v = 0;
  for (int round = 0; round < 50; ++round) {
    EXPECT_TRUE(q.push(round * 2));
    EXPECT_TRUE(q.push(round * 2 + 1));
    ASSERT_TRUE(q.pop(v));
    EXPECT_EQ(v, round * 2);
    ASSERT_TRUE(q.pop(v));
    EXPECT_EQ(v, round * 2 + 1);
  }
}

// Every element arrives exactly once and in order, with the queue regularly full and empty.
TEST(SpscQueue, NothingLostDuplicatedOrReorderedUnderConcurrency)
{
  SpscQueue<long, 64> q;
  constexpr long kCount = 500000;
  std::thread producer([&] {
      for (long i = 1; i <= kCount;) {
        if (q.push(i)) {++i;}
      }
    });
  long expected = 1;
  long v = 0;
  while (expected <= kCount) {
    if (q.pop(v)) {
      ASSERT_EQ(v, expected);
      ++expected;
    }
  }
  producer.join();
  EXPECT_FALSE(q.pop(v));
}
