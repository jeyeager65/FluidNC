// Copyright (c) 2024 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

// Stepping engine that uses direct GPIO accesses timed by spin loops.

#include "Platform.h"
#include "Driver/step_engine.h"
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

static std::atomic<bool>                                  _running(true);
static std::atomic<bool>                                  _pulsing(false);
static uint32_t                                           _elapsed_us = 0;
static uint32_t                                           _target_us  = 0;
static std::mutex                                         mtx;
static std::condition_variable                            cv;
static std::chrono::time_point<std::chrono::steady_clock> _target_time;
static std::chrono::duration<int64_t, std::micro>         _tick_interval;

static bool (*timer_isr_callback)(void);

// Machine time runs this many times faster than real time (the wasm build's fluidnc_set_time_scale); 1 elsewhere
std::atomic<double> g_time_scale { 1.0 };
// Set from inside the step callback (the wasm build's simulated switches): pause stepping this long (real ms) and
// then carry on from now, without catching up - so FluidNC acts on a switch before the next step
std::atomic<int> g_step_hold_ms { 0 };
// Until then (steady_clock ns) machine time runs at 1x whatever the scale: set with g_step_hold_ms, so FluidNC's own
// waits around a switch event (homing's 10 ms dwell after re-arming a switch) stay long enough for the event to arrive
std::atomic<int64_t> g_scale_hold_until { 0 };
double time_scale_now() {
    auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    return now < g_scale_hold_until.load() ? 1.0 : g_time_scale.load();
}

static void timing_loop() {
    while (_running) {
        if (_pulsing) {
            _target_time += std::chrono::duration_cast<std::chrono::nanoseconds>(_tick_interval / time_scale_now());
            auto now = std::chrono::steady_clock::now();
            if (_target_time > now) {
                std::this_thread::sleep_for(_target_time - now);
            }
            if (!timer_isr_callback()) {
                _pulsing = false;
            }
            if (int hold = g_step_hold_ms.exchange(0)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(hold));
                _target_time = std::chrono::steady_clock::now();
            }
        } else {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock);
        }
    }
}
// Call this on shutdown to cleanly stop the timing thread
void stepTimerShutdown() {
    _running = false;
    cv.notify_one();
}

uint32_t stepTimerInit(bool (*callback)(void)) {
    // Note: frequency must be 1000000 (units of microseconds); this implementation uses that frequency
    timer_isr_callback = callback;
    std::thread(timing_loop).detach();
    return STEPPING_FREQUENCY;
}
void stepTimerStart() {
    _tick_interval = std::chrono::microseconds(1);
    _target_time   = std::chrono::steady_clock::now();
    _pulsing       = true;
    cv.notify_one();
}
void stepTimerStop() {
    _pulsing = false;
}

void stepTimerSetTicks(uint32_t ticks) {
    _tick_interval = std::chrono::microseconds(ticks);
};
