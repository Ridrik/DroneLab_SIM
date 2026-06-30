#ifndef DRONELAB_SIMULATION_UTILS_LOCK_FREE_QUEUE_HPP
#define DRONELAB_SIMULATION_UTILS_LOCK_FREE_QUEUE_HPP

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <vector>

#include "dronelab_core_export.h"

namespace {

inline std::size_t nearestPowerOfTwo(std::size_t n) {
    if (n == 0) {
        return 1;
    }
    std::size_t power = 1;
    while (power < n) {
        power <<= 1;  // Shift left to multiply by 2
    }
    return power;
}

}  // namespace

namespace dronelab::sim::concurrency {

enum class Mode : std::uint8_t { kMPMC, kMPSC, kSPMC, kSPSC };

template <Mode M>
[[nodiscard]] consteval bool isSingleConsumer() noexcept {
    return M == Mode::kMPSC || M == Mode::kSPSC;
}

template <Mode M>
[[nodiscard]] consteval bool isSingleProducer() noexcept {
    return M == Mode::kSPMC || M == Mode::kSPSC;
}

template <typename T, Mode M = Mode::kMPMC>
class DRONELAB_CORE_EXPORT LockFreeQueue {
   public:
    struct alignas(64) Cell {
        T item{};
        std::atomic<std::uint64_t> sequence;

        Cell() = default;
        ~Cell() = default;

        Cell(const Cell&) = delete;
        Cell& operator=(const Cell&) = delete;
        Cell(Cell&& other) noexcept : item{std::move(other.item)} {
            sequence.store(other.sequence.load(std::memory_order_relaxed),
                           std::memory_order_relaxed);
        }
        Cell& operator=(Cell&& other) noexcept {
            item = std::move(other.item);
            sequence.store(other.sequence.load(std::memory_order_relaxed),
                           std::memory_order_relaxed);
            return *this;
        }
    };

    explicit LockFreeQueue(std::size_t capacity)
        : capacity_{nearestPowerOfTwo(capacity)}, mask_{capacity_ - 1} {
        cells_.resize(capacity_);
        for (std::size_t i = 0; i < capacity_; ++i) {
            cells_[i].sequence.store(i, std::memory_order_relaxed);
        }
    }
    LockFreeQueue() = default;
    ~LockFreeQueue() = default;

    LockFreeQueue(const LockFreeQueue&) = delete;
    LockFreeQueue& operator=(const LockFreeQueue&) = delete;
    LockFreeQueue(LockFreeQueue&& other) noexcept
        : capacity_{other.capacity_},
          mask_{other.mask_},
          head_{other.head_.load(std::memory_order_relaxed)},
          tail_{other.tail_.load(std::memory_order_relaxed)},
          cells_{std::move(other.cells_)} {};
    LockFreeQueue& operator=(LockFreeQueue&& other) noexcept {
        if (this == &other) {
            return *this;
        }
        capacity_ = other.capacity_;
        mask_ = other.mask_;
        head_.store(other.head_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        tail_.store(other.tail_.load(std::memory_order_relaxed), std::memory_order_relaxed);
        cells_ = std::move(other.cells_);
        return *this;
    };

    template <typename... V>
    bool enqueue(V&&... item) noexcept {  // NOLINT
        auto tail = tail_.load(std::memory_order_relaxed);
        while (true) {
            const auto index = tail & mask_;
            Cell& cell = cells_[index];
            const auto seq = cell.sequence.load(std::memory_order_acquire);
            const bool isFull = seq < tail;
            if (isFull) {
                return false;
            }
            if (seq > tail) {
                tail = tail_.load(std::memory_order_relaxed);
            } else if (tail_.compare_exchange_weak(tail, tail + 1, std::memory_order_release)) {
                cell.item = std::forward<decltype(item)...>(item...);
                cell.sequence.store(tail + 1, std::memory_order_release);
                return true;
            }
        }
    }

    void shrink(std::size_t count) {
        const auto size = getSize();
        if (count >= size) {
            return;
        }
        const auto remove = size - count;
        head_.fetch_add(remove);
    }

    [[nodiscard]] std::optional<std::uint64_t> reserve() noexcept {
        auto tail = tail_.load(std::memory_order_relaxed);
        while (true) {
            const auto index = tail & mask_;
            Cell& cell = cells_[index];
            const auto seq = cell.sequence.load(std::memory_order_acquire);
            const bool isFull = seq < tail;
            if (isFull) {
                return std::nullopt;
            }
            if (seq > tail) {
                tail = tail_.load(std::memory_order_relaxed);
            } else if (tail_.compare_exchange_weak(tail, tail + 1, std::memory_order_release)) {
                return tail;
            }
        }
    }

    [[nodiscard]] std::optional<std::uint64_t> reserveConsumer() noexcept {
        auto head = head_.load(std::memory_order_relaxed);
        while (true) {
            const auto index = head & mask_;
            Cell& cell = cells_[index];
            const auto seq = cell.sequence.load(std::memory_order_acquire);
            const bool isEmpty = seq < head + 1;
            if (isEmpty) {
                return std::nullopt;
            }
            if (head_.compare_exchange_weak(head, head + 1, std::memory_order_release)) {
                return head;
            }
        }
    }

    [[nodiscard]] T* getCell(std::uint64_t ticket) noexcept {
        return &cells_[ticket & mask_].item;
    }

    [[nodiscard]] T* getFront()
        requires(isSingleConsumer<M>())
    {
        const auto head = head_.load(std::memory_order_relaxed);
        auto& headCell = cells_[head & mask_];
        const auto seq = headCell.sequence.load(std::memory_order_acquire);
        const bool isEmpty = seq <= head;
        if (isEmpty) {
            return nullptr;
        }
        return &headCell.item;
    }

    void commit(std::uint64_t ticket) noexcept {
        cells_[ticket & mask_].sequence.store(ticket + 1, std::memory_order_release);
    }

    void commitConsumer(std::uint64_t ticket) noexcept {
        cells_[ticket & mask_].sequence.store(ticket + capacity_, std::memory_order_release);
    }

    [[nodiscard]] bool dequeue(T& value) noexcept {
        auto head = head_.load(std::memory_order_relaxed);
        while (true) {
            const auto index = head & mask_;
            Cell& cell = cells_[index];
            const auto seq = cell.sequence.load(std::memory_order_acquire);
            const bool isEmpty = seq <= head;
            if (isEmpty) {
                return false;
            }
            if (head_.compare_exchange_weak(head, head + 1, std::memory_order_release)) {
                value = std::move(cell.item);
                cell.sequence.store(head + capacity_, std::memory_order_release);
                return true;
            }
        }
    }

    bool dequeue() noexcept {
        auto head = head_.load(std::memory_order_relaxed);
        while (true) {
            const auto index = head & mask_;
            Cell& cell = cells_[index];
            const auto seq = cell.sequence.load(std::memory_order_acquire);
            const bool isEmpty = seq <= head;
            if (isEmpty) {
                return false;
            }
            if (head_.compare_exchange_weak(head, head + 1, std::memory_order_release)) {
                cell.sequence.store(head + capacity_, std::memory_order_release);
                return true;
            }
        }
    }

    [[nodiscard]] std::size_t getCapacity() const noexcept {
        return capacity_;
    }

    [[nodiscard]] std::size_t getSize() const noexcept {
        return tail_.load(std::memory_order_relaxed) - head_.load(std::memory_order_relaxed);
    }

    [[nodiscard]] bool isEmpty() const noexcept {
        return getSize() == 0;
    }

    [[nodiscard]] bool isFull() const noexcept {
        return getSize() >= capacity_;
    }

   private:
    std::size_t capacity_;
    std::size_t mask_;  // Mask for index calculation
    alignas(64) std::atomic<std::uint64_t> head_{0};
    alignas(64) std::atomic<std::uint64_t> tail_{0};
    alignas(64) std::vector<Cell> cells_{};
};

}  // namespace dronelab::sim::concurrency

#endif  // DRONELAB_SIMULATION_UTILS_LOCK_FREE_QUEUE_HPP