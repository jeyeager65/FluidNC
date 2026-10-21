// Simulated switches for the wasm build (lr4-sandbox's machine in the browser). There is no real GPIO here:
// capture/gpio.cpp's gpio_read() returns the bits of g_virtual_gpios, and every gpio_write() comes through
// g_gpio_write_hook. That hook counts each registered motor's steps from its step / direction pins (in the step loop,
// so a switch trips on the exact step), and sets a switch's pin when its motor passes the trip point:
//   fluidnc_set_switch(gpio, step_pin, dir_pin, trip_steps, active_below)
//       gpio reads high while the motor's step count is <= trip_steps (active_below) or >= trip_steps (otherwise).
//       Limit switches (each Y motor its own = squaring) and the touch plate (the Z motor's count) alike.
//   fluidnc_clear_switch(gpio)        the switch is gone: its pin stays low (a disconnected switch)
//   fluidnc_motor_steps(step_pin)     a motor's step count since boot (+ = the positive direction)
//   fluidnc_pin_mask()                the pin levels, bit n = gpio n (a double: 49 bits fit)
//   fluidnc_set_time_scale(k)         machine time k x real time (stepping, FluidNC's loops and delays: settle,
//                                     G4); around each switch change it runs at 1x for a moment (StepTimer.cpp)
// Pins are active high: the config (lr4-sandbox's sim_config.js) names them without :low.

#include <emscripten.h>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <mutex>
#include <thread>

#include "Platform.h"             // MAX_N_GPIO
#include "Driver/fluidnc_gpio.h"  // pinnum_t

extern std::atomic<uint64_t> g_virtual_gpios;
extern void (*g_gpio_write_hook)(pinnum_t pin, bool value);
extern std::atomic<double>   g_time_scale;
extern std::atomic<int>      g_step_hold_ms;
extern std::atomic<int64_t>  g_scale_hold_until;

namespace {
    struct Switch {
        int32_t gpio = -1;
        int32_t step = -1;
        int32_t trip = 0;
        bool    below = true;
    };
    const int       MAX_SWITCHES = 16;
    Switch          switches[MAX_SWITCHES];
    std::mutex      switch_mutex;  // set / clear from JS; the hook reads without it (int fields, benign)
    bool            level[MAX_N_GPIO + 1];
    int32_t         dir_of[MAX_N_GPIO + 1];   // step pin -> its direction pin (-1: not a motor)
    std::atomic<int32_t> steps[MAX_N_GPIO + 1];  // step pin -> count

    // -> true if the pin changed
    bool update(const Switch& s) {
        if (s.gpio < 0) {
            return false;
        }
        int32_t  n      = steps[s.step].load();
        bool     active = s.below ? n <= s.trip : n >= s.trip;
        uint64_t bit    = uint64_t(1) << s.gpio;
        uint64_t was    = active ? g_virtual_gpios.fetch_or(bit) : g_virtual_gpios.fetch_and(~bit);
        return ((was & bit) != 0) != active;
    }

    // FluidNC's StandardStepper drives the direction pin high for the negative direction
    void on_write(pinnum_t pin, bool value) {
        if (pin < 0 || pin > MAX_N_GPIO) {
            return;
        }
        bool rising = value && !level[pin];
        level[pin]  = value;
        if (!rising || dir_of[pin] < 0) {
            return;
        }
        steps[pin].fetch_add(level[dir_of[pin]] ? -1 : 1);
        bool changed = false;
        for (auto& s : switches) {
            if (s.step == int32_t(pin)) {
                changed |= update(s);
            }
        }
        // a switch changed: the step loop pauses (real time, no catch-up after) while FluidNC's polling loop sends the
        // pin event and the protocol loop acts on it (a homing motor is then locked out): it stops on this step at
        // any time scale
        if (changed) {
            g_step_hold_ms.store(10);
            g_scale_hold_until.store((std::chrono::steady_clock::now() + std::chrono::milliseconds(30)).time_since_epoch().count());
        }
    }

    void ensure_hook() {
        static bool once = false;
        if (!once) {
            once = true;
            for (auto& d : dir_of) {
                d = -1;
            }
            g_gpio_write_hook = on_write;
        }
    }
}

extern "C" {

EMSCRIPTEN_KEEPALIVE
void fluidnc_set_switch(int gpio, int step_pin, int dir_pin, int trip_steps, int active_below) {
    if (gpio < 0 || gpio > MAX_N_GPIO || step_pin < 0 || step_pin > MAX_N_GPIO || dir_pin < 0 || dir_pin > MAX_N_GPIO) {
        return;
    }
    std::lock_guard<std::mutex> lock(switch_mutex);
    ensure_hook();
    dir_of[step_pin] = dir_pin;
    Switch* slot     = nullptr;
    for (auto& s : switches) {
        if (s.gpio == gpio) {
            slot = &s;
            break;
        }
    }
    for (auto& s : switches) {
        if (!slot && s.gpio < 0) {
            slot = &s;
        }
    }
    if (!slot) {
        return;  // full
    }
    slot->step  = step_pin;
    slot->trip  = trip_steps;
    slot->below = active_below != 0;
    slot->gpio  = gpio;
    update(*slot);
}

EMSCRIPTEN_KEEPALIVE
void fluidnc_clear_switch(int gpio) {
    std::lock_guard<std::mutex> lock(switch_mutex);
    for (auto& s : switches) {
        if (s.gpio == gpio) {
            s.gpio = -1;
            s.step = -1;
        }
    }
    if (gpio >= 0 && gpio <= MAX_N_GPIO) {
        g_virtual_gpios.fetch_and(~(uint64_t(1) << gpio));
    }
}

// register a motor (step + direction pin) so its steps are counted even with no switch on it
EMSCRIPTEN_KEEPALIVE
void fluidnc_add_motor(int step_pin, int dir_pin) {
    if (step_pin < 0 || step_pin > MAX_N_GPIO || dir_pin < 0 || dir_pin > MAX_N_GPIO) {
        return;
    }
    std::lock_guard<std::mutex> lock(switch_mutex);
    ensure_hook();
    dir_of[step_pin] = dir_pin;
}

EMSCRIPTEN_KEEPALIVE
int fluidnc_motor_steps(int step_pin) {
    return step_pin >= 0 && step_pin <= MAX_N_GPIO ? steps[step_pin].load() : 0;
}

EMSCRIPTEN_KEEPALIVE
double fluidnc_pin_mask() {
    return double(g_virtual_gpios.load());
}

EMSCRIPTEN_KEEPALIVE
void fluidnc_set_time_scale(double k) {
    g_time_scale.store(k > 0.01 ? k : 0.01);
}

}  // extern "C"
