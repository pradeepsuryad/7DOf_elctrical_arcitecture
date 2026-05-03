#pragma once
#include "hal.hpp"
#include <deque>
#include <vector>

// ── FakeCanBus ────────────────────────────────────────────────────────────────
//
// In-memory implementation of hal::ICanBus for unit tests.
//
// How it works:
//   tx_queue  — frames transmitted by the controller end up here.
//               The test pops them and inspects the bytes.
//   rx_queue  — the test pushes frames here to simulate incoming feedback.
//               The controller calls receive() and drains them.
//
// No threading — this runs single-threaded in test context.
// ─────────────────────────────────────────────────────────────────────────────

namespace test {

class FakeCanBus : public hal::ICanBus {
public:
    // ── ICanBus interface ─────────────────────────────────────────────────────

    bool transmit(const hal::CanFrame& frame) override {
        if (inject_tx_error_) return false;
        tx_queue_.push_back(frame);
        return true;
    }

    bool receive(hal::CanFrame& frame) override {
        if (rx_queue_.empty()) return false;
        frame = rx_queue_.front();
        rx_queue_.pop_front();
        return true;
    }

    bool is_healthy() const override { return healthy_; }

    // ── Test helpers ──────────────────────────────────────────────────────────

    // Pop and return the next frame the controller transmitted.
    // Throws std::runtime_error in release; use ASSERT_FALSE(tx_empty()) first.
    hal::CanFrame pop_tx() {
        hal::CanFrame f = tx_queue_.front();
        tx_queue_.pop_front();
        return f;
    }

    // Push a frame as if it arrived from the joint over CAN.
    void push_rx(const hal::CanFrame& frame) {
        rx_queue_.push_back(frame);
    }

    bool tx_empty() const { return tx_queue_.empty(); }
    size_t tx_count() const { return tx_queue_.size(); }
    void clear_tx() { tx_queue_.clear(); }

    // Inject a failure on the next transmit() call.
    void set_tx_error(bool v) { inject_tx_error_ = v; }
    void set_healthy(bool v)  { healthy_ = v; }

private:
    std::deque<hal::CanFrame> tx_queue_;
    std::deque<hal::CanFrame> rx_queue_;
    bool inject_tx_error_{false};
    bool healthy_{true};
};

}  // namespace test
