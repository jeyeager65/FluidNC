// Console channel for the wasm port. Mirrors posix/Console.cpp's use of
// Lineedit (src/lineedit.h) for local echo, intra-line editing (arrow
// keys, backspace, word motion), history, and $-setting tab completion --
// same as a real serial console -- except there's no tty here, so input
// arrives as whatever raw bytes JS forwards (from e.g. an xterm.js
// onData callback) into a small queue that read()/available() drain.
//
// This deliberately does NOT use Channel::push(): push() checks
// is_realtime_command() directly with no way for a Channel subclass to
// veto it, whereas Channel::pollLine()'s read()-driven loop checks
// realtimeOkay(ch) && is_realtime_command(ch) -- realtimeOkay() is the
// hook Lineedit needs to say "this ~ is actually the tail of the Delete
// key's ESC[3~ sequence, not a realtime Cycle-Start command". Using
// push() would let that ~ get intercepted as realtime before Lineedit's
// escape-sequence parser ever saw it, breaking the Delete key. Only the
// read()/pollLine() path consults realtimeOkay() first, so input has to
// flow through a real read() the way posix/Console.cpp's does from an
// actual tty.

#include <emscripten.h>
#include <cstring>
#include <deque>
#include <atomic>
#include <mutex>
#include <string>

#include "Channel.h"
#include "Serial.h"  // allChannels
#include "Driver/Console.h"
#include "lineedit.h"

// write() runs on the FluidNC background thread (its own pthread/Worker),
// where `self` is that worker's own global object, not the page's window --
// plain EM_JS/EM_ASM there would silently write to the wrong `self` and
// never reach the page. MAIN_THREAD_EM_ASM proxies the call to actually run
// on the main thread regardless of which pthread it's invoked from.
//
// Queued mode (fluidnc_output_queue(1), lr4-sandbox's page): write() only appends to a buffer and the page takes it
// with fluidnc_take_output() on its own timer - MAIN_THREAD_EM_ASM waits for the page's main thread, so a busy page
// (drawing a frame) would stall FluidNC's main loop, the step segment buffer would run dry and a homing approach would
// stop short (cycle stop = "Homing Fail Approach"), worst at a high time scale.
namespace {
    std::atomic<bool> output_queued { false };
    std::mutex        output_mutex;
    std::string       output_pending, output_taken;
}

extern "C" {
EMSCRIPTEN_KEEPALIVE
void fluidnc_output_queue(int on) {
    output_queued.store(on != 0);
}
// everything written since the last call (UTF-8, NUL-terminated; valid until the next call)
EMSCRIPTEN_KEEPALIVE
const char* fluidnc_take_output() {
    std::lock_guard<std::mutex> lock(output_mutex);
    output_taken.clear();
    output_taken.swap(output_pending);
    return output_taken.c_str();
}
}

void wasm_console_write(const char* buf, int len) {
    if (output_queued.load()) {
        std::lock_guard<std::mutex> lock(output_mutex);
        output_pending.append(buf, len);
        return;
    }
    MAIN_THREAD_EM_ASM(
        {
            const text = UTF8ToString($0, $1);
            if (typeof self.fluidncOnOutput === 'function') {
                self.fluidncOnOutput(text);
            } else {
                console.log(text);
            }
        },
        buf,
        len);
}

namespace {
class WasmConsole : public Channel {
private:
    Lineedit* _lineedit;

    std::mutex        _rx_mutex;
    std::deque<uint8_t> _rx_queue;

public:
    WasmConsole() : Channel("WasmConsole", false) {}

    void init() override {
        _lineedit = new Lineedit(this, _line, Channel::maxLine - 1);
        allChannels.registration(this);
    }
    void flushRx() override {
        std::lock_guard<std::mutex> lock(_rx_mutex);
        _rx_queue.clear();
    }

    size_t write(uint8_t c) override {
        char ch = (char)c;
        wasm_console_write(&ch, 1);
        return 1;
    }
    size_t write(const uint8_t* buffer, size_t size) override {
        wasm_console_write(reinterpret_cast<const char*>(buffer), (int)size);
        return size;
    }

    // Called from fluidnc_send_text() (wasm_main.cpp), which runs on the
    // main JS thread; read()/available() run on the FluidNC background
    // thread via Channel::pollLine() -- hence the mutex.
    void receive(const uint8_t* data, size_t len) {
        std::lock_guard<std::mutex> lock(_rx_mutex);
        for (size_t i = 0; i < len; ++i) {
            _rx_queue.push_back(data[i]);
        }
    }

    int available() override {
        std::lock_guard<std::mutex> lock(_rx_mutex);
        return (int)_rx_queue.size();
    }
    int read() override {
        std::lock_guard<std::mutex> lock(_rx_mutex);
        if (_rx_queue.empty()) {
            return -1;
        }
        uint8_t c = _rx_queue.front();
        _rx_queue.pop_front();
        return c;
    }
    int peek() override {
        std::lock_guard<std::mutex> lock(_rx_mutex);
        return _rx_queue.empty() ? -1 : _rx_queue.front();
    }

    // Same pattern as PosixConsole: Lineedit decides whether a character
    // should be treated as a GRBL realtime command (it isn't, e.g., in the
    // middle of an escape sequence) and drives line assembly itself
    // (editing, history, completion) instead of the base class's plain
    // accumulate-until-newline.
    bool realtimeOkay(char c) override { return _lineedit->realtime(c); }

    bool lineComplete(char* line, char c) override {
        if (_lineedit->step(c)) {
            _linelen        = _lineedit->finish();
            _line[_linelen] = '\0';
            strcpy(line, _line);
            _linelen = 0;
            return true;
        }
        return false;
    }
};

WasmConsole wasmConsole;
}  // namespace

Channel& Console = wasmConsole;

extern "C" {
// Bridge target for fluidnc_send_text() (wasm_main.cpp) -- kept here
// rather than reaching into WasmConsole's private state from another file.
void wasm_console_receive(const uint8_t* data, size_t len) {
    wasmConsole.receive(data, len);
}
}
