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

#ifndef ROBOT_SERVER_COMM__CORE__BOUNDED_QUEUE_HPP_
#define ROBOT_SERVER_COMM__CORE__BOUNDED_QUEUE_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace robot_server_comm
{

/// Thread safe FIFO queue limited both in number of items and in bytes.
///
/// When a new item does not fit, the OLDEST items are discarded to make room.
/// For live sensor data this is the right policy: when the link is slow or
/// down, recent measurements are worth more than stale ones, and memory use
/// stays bounded whatever happens to the network.
template<typename ItemT>
class BoundedQueue
{
public:
  /// Outcome of push().
  struct PushResult
  {
    bool accepted{false};          ///< False when the item was refused (too large or closed).
    std::size_t dropped_items{0};  ///< Old items discarded to make room.
  };

  /// \param max_items largest number of queued items (at least 1)
  /// \param max_bytes largest total size of queued items (at least 1)
  BoundedQueue(std::size_t max_items, std::size_t max_bytes)
  : max_items_(max_items == 0 ? 1 : max_items),
    max_bytes_(max_bytes == 0 ? 1 : max_bytes)
  {
  }

  /// Adds an item without ever blocking the caller.
  /// \param item value to queue
  /// \param item_bytes size accounted for this item
  PushResult push(ItemT item, std::size_t item_bytes)
  {
    PushResult result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_ || item_bytes > max_bytes_) {
        return result;
      }
      while (!items_.empty() &&
        (items_.size() >= max_items_ || bytes_ + item_bytes > max_bytes_))
      {
        bytes_ -= items_.front().second;
        items_.pop_front();
        ++result.dropped_items;
      }
      bytes_ += item_bytes;
      items_.emplace_back(std::move(item), item_bytes);
      result.accepted = true;
    }
    not_empty_.notify_one();
    return result;
  }

  /// Removes the oldest item, waiting up to `timeout` for one to arrive.
  /// Returns an empty optional on timeout or when the queue is closed and empty.
  std::optional<ItemT> pop(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_.wait_for(lock, timeout, [this] {return closed_ || !items_.empty();});
    if (items_.empty()) {
      return std::nullopt;
    }
    std::optional<ItemT> item(std::move(items_.front().first));
    bytes_ -= items_.front().second;
    items_.pop_front();
    return item;
  }

  /// Refuses new items and wakes up every waiting consumer.
  void close()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    not_empty_.notify_all();
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return items_.size();
  }

  std::size_t bytes() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return bytes_;
  }

private:
  const std::size_t max_items_;
  const std::size_t max_bytes_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_;
  std::deque<std::pair<ItemT, std::size_t>> items_;
  std::size_t bytes_{0};
  bool closed_{false};
};

}  // namespace robot_server_comm

#endif  // ROBOT_SERVER_COMM__CORE__BOUNDED_QUEUE_HPP_
