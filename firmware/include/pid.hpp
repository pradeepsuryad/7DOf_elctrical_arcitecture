#pragma once
#include <algorithm>
#include <cmath>

// ── PID controller ────────────────────────────────────────────────────────────
//
// Template type T is the numeric type: float for MCU (FPU-backed),
// double for desktop simulation / HIL where precision matters more than speed.
//
// Anti-windup strategy: clamp the integral term to [output_min, output_max].
// This prevents integrator windup when the actuator saturates (e.g. motor at
// max torque) — without it, the integral keeps growing and causes large
// overshoot when the saturating condition clears.
//
// Derivative filtering: raw d(error)/dt amplifies high-frequency sensor noise.
// We apply a first-order IIR low-pass on the derivative:
//   d_filtered = alpha * d_raw + (1 - alpha) * d_prev
//   alpha = dt / (tau_d + dt)   where tau_d is the filter time constant.
// ─────────────────────────────────────────────────────────────────────────────

namespace control {

template <typename T = float>
class Pid {
public:
    struct Gains {
        T kp{0};    // proportional gain
        T ki{0};    // integral gain
        T kd{0};    // derivative gain
        T tau_d{static_cast<T>(0.005)};  // derivative low-pass time constant (s)
        T out_min{static_cast<T>(-1)};   // output clamp lower bound
        T out_max{static_cast<T>( 1)};   // output clamp upper bound
    };

    explicit Pid(const Gains& g) : g_(g) {}

    // Call at a fixed rate (dt seconds between calls).
    // Returns the control output clamped to [out_min, out_max].
    T update(T setpoint, T measurement, T dt) {
        if (dt <= T{0}) return prev_output_;

        const T error = setpoint - measurement;

        // Proportional
        const T p = g_.kp * error;

        // Integral with anti-windup clamp
        integral_ += g_.ki * error * dt;
        integral_ = std::clamp(integral_, g_.out_min, g_.out_max);

        // Derivative (filtered)
        const T d_raw = (error - prev_error_) / dt;
        const T alpha  = dt / (g_.tau_d + dt);
        d_filtered_ = alpha * d_raw + (T{1} - alpha) * d_filtered_;
        const T d = g_.kd * d_filtered_;

        prev_error_ = error;

        const T output = std::clamp(p + integral_ + d, g_.out_min, g_.out_max);
        prev_output_ = output;
        return output;
    }

    void reset() {
        integral_   = T{0};
        prev_error_ = T{0};
        d_filtered_ = T{0};
        prev_output_= T{0};
    }

    void set_gains(const Gains& g) { g_ = g; reset(); }
    const Gains& gains() const     { return g_; }

    T integral()    const { return integral_; }
    T prev_error()  const { return prev_error_; }

private:
    Gains g_;
    T integral_   {0};
    T prev_error_ {0};
    T d_filtered_ {0};
    T prev_output_{0};
};

}  // namespace control
