// Minimal Win32 window plus the input state a fly camera needs.
#pragma once

#include "core/Common.h"

namespace ga {

struct InputState {
    bool keyDown[256] = {};
    bool keyPressed[256] = {};   // edge-triggered, cleared by Window::NewFrame
    float mouseDx = 0, mouseDy = 0;  // accumulated while EITHER button drags
    float mouseX = 0, mouseY = 0;    // live cursor position, client pixels
    float wheel = 0;
    bool rmb = false;
    bool lmb = false;                // Google-Earth gestures: plain = grab-pan,
                                     // SHIFT = tilt orbit, ALT = rotate orbit (main decides)
};

class Window {
public:
    bool Create(uint32_t width, uint32_t height, const wchar_t* title);
    void Destroy();

    // Drains the message queue. Returns false once the window wants to close.
    bool PumpMessages();
    void NewFrame();          // clears per-frame edge state (pressed, deltas)

    // The window title doubles as the HUD (sim clock, tide height, time scale): zero rendering
    // dependencies, readable in every capture tool, and PIX shows it in the process list.
    void SetTitle(const wchar_t* title) { if (m_hwnd) SetWindowTextW(m_hwnd, title); }

    HWND Handle() const { return m_hwnd; }
    uint32_t Width() const { return m_width; }
    uint32_t Height() const { return m_height; }
    bool TakeResized() { bool r = m_resized; m_resized = false; return r; }
    const InputState& Input() const { return m_input; }

private:
    static LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
    LRESULT Handle(UINT msg, WPARAM wp, LPARAM lp);

    HWND m_hwnd = nullptr;
    uint32_t m_width = 0, m_height = 0;
    bool m_closing = false;
    bool m_resized = false;
    InputState m_input;
    POINT m_lastCursor{};
    bool m_haveCursor = false;
};

}  // namespace ga
