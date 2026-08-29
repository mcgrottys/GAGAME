#include "core/Window.h"

#include <windowsx.h>   // GET_X_LPARAM / GET_Y_LPARAM / GET_WHEEL_DELTA_WPARAM

namespace ga {

static const wchar_t* kClassName = L"gagameWindow";

bool Window::Create(uint32_t width, uint32_t height, const wchar_t* title) {
    WNDCLASSEXW wc{};
    wc.cbSize = sizeof(wc);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = &Window::WndProc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.lpszClassName = kClassName;
    if (!RegisterClassExW(&wc) && GetLastError() != ERROR_CLASS_ALREADY_EXISTS) {
        Log("[window] RegisterClassEx failed (%lu)", GetLastError());
        return false;
    }

    RECT r{0, 0, static_cast<LONG>(width), static_cast<LONG>(height)};
    AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    m_hwnd = CreateWindowExW(0, kClassName, title, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT,
                             CW_USEDEFAULT, r.right - r.left, r.bottom - r.top, nullptr, nullptr,
                             wc.hInstance, this);
    if (!m_hwnd) {
        Log("[window] CreateWindowEx failed (%lu)", GetLastError());
        return false;
    }
    m_width = width;
    m_height = height;
    ShowWindow(m_hwnd, SW_SHOW);
    return true;
}

void Window::Destroy() {
    if (m_hwnd) { DestroyWindow(m_hwnd); m_hwnd = nullptr; }
}

LRESULT CALLBACK Window::WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    if (msg == WM_NCCREATE) {
        auto* cs = reinterpret_cast<CREATESTRUCTW*>(lp);
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
    }
    auto* self = reinterpret_cast<Window*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    if (self) {
        LRESULT handled = self->Handle(msg, wp, lp);
        if (handled != -1) return handled;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

LRESULT Window::Handle(UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_CLOSE:
        case WM_DESTROY:
            m_closing = true;
            return 0;
        case WM_SIZE: {
            const uint32_t w = LOWORD(lp), h = HIWORD(lp);
            if (w && h && (w != m_width || h != m_height)) {
                m_width = w;
                m_height = h;
                m_resized = true;
            }
            return 0;
        }
        case WM_KEYDOWN:
        case WM_SYSKEYDOWN:
            if (wp < 256) {
                if (!m_input.keyDown[wp]) m_input.keyPressed[wp] = true;
                m_input.keyDown[wp] = true;
            }
            if (wp == VK_ESCAPE) m_closing = true;
            return 0;
        case WM_KEYUP:
        case WM_SYSKEYUP:
            if (wp < 256) m_input.keyDown[wp] = false;
            return 0;
        case WM_RBUTTONDOWN:
            m_input.rmb = true;
            m_haveCursor = false;
            SetCapture(m_hwnd);
            return 0;
        case WM_RBUTTONUP:
            m_input.rmb = false;
            if (!m_input.lmb) ReleaseCapture();
            return 0;
        case WM_LBUTTONDOWN:
            m_input.lmb = true;
            m_haveCursor = false;
            SetCapture(m_hwnd);
            return 0;
        case WM_LBUTTONUP:
            m_input.lmb = false;
            if (!m_input.rmb) ReleaseCapture();
            return 0;
        case WM_MOUSEMOVE: {
            POINT p{static_cast<LONG>(GET_X_LPARAM(lp)), static_cast<LONG>(GET_Y_LPARAM(lp))};
            m_input.mouseX = static_cast<float>(p.x);
            m_input.mouseY = static_cast<float>(p.y);
            if (m_input.rmb || m_input.lmb) {
                if (m_haveCursor) {
                    m_input.mouseDx += static_cast<float>(p.x - m_lastCursor.x);
                    m_input.mouseDy += static_cast<float>(p.y - m_lastCursor.y);
                }
                m_lastCursor = p;
                m_haveCursor = true;
            }
            return 0;
        }
        case WM_MOUSEWHEEL:
            m_input.wheel += static_cast<float>(GET_WHEEL_DELTA_WPARAM(wp)) / WHEEL_DELTA;
            return 0;
        case WM_SYSCOMMAND:
            // ALT is the rotate-orbit modifier; without this, a bare ALT tap enters the Win32
            // menu loop (focus steal + beep on the next key).
            if ((wp & 0xFFF0) == SC_KEYMENU) return 0;
            break;
        default:
            break;
    }
    return -1;
}

bool Window::PumpMessages() {
    MSG msg;
    while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return !m_closing;
}

void Window::NewFrame() {
    memset(m_input.keyPressed, 0, sizeof(m_input.keyPressed));
    m_input.mouseDx = m_input.mouseDy = 0;
    m_input.wheel = 0;
}

}  // namespace ga
