// Copyright 2026 Anas Derkaoui
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "robot_server_comm/core/bounded_queue.hpp"

namespace robot_server_comm
{

using std::chrono::milliseconds;

TEST(BoundedQueue, DeliversItemsInArrivalOrder)
{
  BoundedQueue<int> queue(10, 1000);
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(queue.push(i, 1).accepted);
  }
  EXPECT_EQ(5u, queue.size());
  EXPECT_EQ(5u, queue.bytes());
  for (int i = 0; i < 5; ++i) {
    const auto item = queue.pop(milliseconds(0));
    ASSERT_TRUE(item.has_value());
    EXPECT_EQ(i, *item);
  }
  EXPECT_EQ(0u, queue.size());
  EXPECT_EQ(0u, queue.bytes());
}

TEST(BoundedQueue, DropsTheOldestItemWhenTheItemLimitIsReached)
{
  BoundedQueue<int> queue(3, 1000);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(0u, queue.push(i, 1).dropped_items);
  }
  const auto result = queue.push(3, 1);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(1u, result.dropped_items);
  EXPECT_EQ(3u, queue.size());
  // 0 was the oldest: it is gone, the newest data is kept.
  EXPECT_EQ(1, *queue.pop(milliseconds(0)));
  EXPECT_EQ(2, *queue.pop(milliseconds(0)));
  EXPECT_EQ(3, *queue.pop(milliseconds(0)));
}

TEST(BoundedQueue, DropsAsManyOldItemsAsNeededToRespectTheByteLimit)
{
  BoundedQueue<int> queue(100, 10);
  EXPECT_TRUE(queue.push(1, 4).accepted);
  EXPECT_TRUE(queue.push(2, 4).accepted);
  const auto result = queue.push(3, 8);
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(2u, result.dropped_items);
  EXPECT_EQ(1u, queue.size());
  EXPECT_EQ(8u, queue.bytes());
  EXPECT_EQ(3, *queue.pop(milliseconds(0)));
}

TEST(BoundedQueue, RefusesAnItemLargerThanTheWholeBudget)
{
  BoundedQueue<int> queue(100, 10);
  EXPECT_TRUE(queue.push(1, 4).accepted);
  const auto result = queue.push(2, 11);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(0u, result.dropped_items);
  // The item already queued was not sacrificed for nothing.
  EXPECT_EQ(1u, queue.size());
}

TEST(BoundedQueue, PopTimesOutWhenEmpty)
{
  BoundedQueue<int> queue(4, 100);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(queue.pop(milliseconds(50)).has_value());
  EXPECT_GE(std::chrono::steady_clock::now() - start, milliseconds(40));
}

TEST(BoundedQueue, CloseWakesAWaitingConsumerAndRefusesNewItems)
{
  BoundedQueue<int> queue(4, 100);
  std::atomic<bool> returned{false};
  std::thread consumer([&queue, &returned] {
      EXPECT_FALSE(queue.pop(milliseconds(10000)).has_value());
      returned.store(true);
    });
  std::this_thread::sleep_for(milliseconds(50));
  queue.close();
  consumer.join();
  EXPECT_TRUE(returned.load());
  EXPECT_FALSE(queue.push(1, 1).accepted);
}

TEST(BoundedQueue, ItemsQueuedBeforeCloseCanStillBeRead)
{
  BoundedQueue<int> queue(4, 100);
  queue.push(7, 1);
  queue.close();
  const auto item = queue.pop(milliseconds(0));
  ASSERT_TRUE(item.has_value());
  EXPECT_EQ(7, *item);
  EXPECT_FALSE(queue.pop(milliseconds(0)).has_value());
}

TEST(BoundedQueue, SupportsOneProducerAndOneConsumerThread)
{
  constexpr int kCount = 20000;
  BoundedQueue<int> queue(kCount, static_cast<std::size_t>(kCount));
  std::vector<int> received;
  received.reserve(kCount);

  std::thread consumer([&queue, &received] {
      while (static_cast<int>(received.size()) < kCount) {
        const auto item = queue.pop(milliseconds(1000));
        if (!item.has_value()) {
          break;
        }
        received.push_back(*item);
      }
    });
  for (int i = 0; i < kCount; ++i) {
    ASSERT_TRUE(queue.push(i, 1).accepted);
  }
  consumer.join();

  ASSERT_EQ(static_cast<std::size_t>(kCount), received.size());
  for (int i = 0; i < kCount; ++i) {
    ASSERT_EQ(i, received[static_cast<std::size_t>(i)]);
  }
}

}  // namespace robot_server_comm
