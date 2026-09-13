// FramePipe - the rail recorder, moved verbatim from main.cpp (M12 step 1a).
#pragma once

#include "core/Common.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace ga::app {

// ================================================================================================
//  M9s: THE RECORDER, ENCODING AS IT GOES.
//
//  A rail used to write 1200 PNGs and then run ffmpeg over them. The rail metrics priced that
//  honestly for the first time: 1.7 ms to RENDER a frame, ~90 ms to PNG it. Fifty-three parts
//  in fifty-four of a recording's wall clock went into compressing intermediates that were
//  deleted immediately afterwards -- and it also meant the mp4 could not exist until every
//  frame had hit the disk twice.
//
//  So the frames go straight into an encoder over a pipe: no PNG, no intermediate directory,
//  no second pass. NVENC where the GPU offers it (it is already rendering; the encoder blocks
//  are idle), libx264 where it does not. rawvideo in, mp4 out, one process for the whole rail.
//
//  Deliberately ffmpeg over a pipe rather than linking an encoder: it keeps the codec choice
//  a runtime decision, costs nothing when unused, and a recorder that shells out is a recorder
//  that cannot corrupt the renderer.
// ================================================================================================
class FramePipe {
public:
    bool Open(const std::string& path, uint32_t w, uint32_t h, uint32_t fps) {
        m_w = w;
        m_h = h;
        const Codec& c = PickCodec();
        char cmd[1024];
        snprintf(cmd, sizeof(cmd),
                 "ffmpeg -hide_banner -loglevel warning -y -f rawvideo -pixel_format rgba "
                 "-video_size %ux%u -framerate %u -i - %s -pix_fmt yuv420p \"%s\"",
                 w, h, fps, c.args, path.c_str());
        m_f = _popen(cmd, "wb");
        if (!m_f) {
            Log("[rec] could not start ffmpeg -- falling back to PNG frames");
            return false;
        }
        Log("[rec] encoding %ux%u @%u fps straight from the framebuffer via %s -> %s", w, h, fps,
            c.label, path.c_str());
        return true;
    }
    bool Open() const { return m_f != nullptr; }

    // The readback row pitch is aligned and usually exceeds width*4, so rows are written one at
    // a time. Writing the whole buffer would shear the picture by the padding, every frame.
    void Write(const std::vector<uint8_t>& px, uint32_t rowPitch) {
        if (!m_f) return;
        const size_t row = size_t(m_w) * 4;
        size_t wrote = 0;
        for (uint32_t y = 0; y < m_h; ++y) {
            const size_t off = size_t(y) * rowPitch;
            if (off + row > px.size()) break;
            wrote += fwrite(px.data() + off, 1, row, m_f);
        }
        if (!m_reported) {
            m_reported = true;
            Log("[rec] first frame: %zu px bytes, pitch %u, wrote %zu of %zu expected%s",
                px.size(), rowPitch, wrote, row * m_h,
                ferror(m_f) ? " -- PIPE ERROR" : "");
        }
    }
    void Close() {
        if (!m_f) return;
        _pclose(m_f);
        m_f = nullptr;
        Log("[rec] encoder closed");
    }

private:
    struct Codec {
        const char* args;
        const char* label;
    };

    // PROBE BY ENCODING, NOT BY LISTING. The first version of this asked `ffmpeg -encoders` for
    // h264_nvenc, found it, and produced a ZERO-BYTE mp4: the encoder is compiled in and listed,
    // but this machine's driver exposes NVENC API 13.0 while that ffmpeg build requires 13.1, so
    // it fails at open. "Is it listed" and "does it work" are different questions and only the
    // second one matters -- so the probe actually encodes one frame to null and checks the exit
    // status. It costs a fraction of a second, once, and it means the day the driver is updated
    // NVENC starts being used with no code change at all.
    static const Codec& PickCodec() {
        static const Codec kCandidates[] = {
            {"-c:v h264_nvenc -preset p5 -rc vbr -cq 21 -b:v 0", "h264_nvenc (NVIDIA GPU)"},
            {"-c:v h264_qsv -global_quality 21", "h264_qsv (Intel GPU)"},
            {"-c:v h264_amf -quality balanced -rc cqp -qp_i 21 -qp_p 21", "h264_amf (AMD GPU)"},
            {"-c:v libx264 -preset veryfast -crf 20", "libx264 (CPU)"},
        };
        static const Codec* chosen = nullptr;
        if (chosen) return *chosen;
        for (const Codec& c : kCandidates) {
            char probe[512];
            snprintf(probe, sizeof(probe),
                     "ffmpeg -hide_banner -loglevel error -f lavfi "
                     "-i color=c=black:s=64x64:d=0.1 -frames:v 1 %s -f null - >nul 2>&1",
                     c.args);
            if (system(probe) == 0) {
                chosen = &c;
                if (&c != &kCandidates[0]) {
                    Log("[rec] %s unavailable here (encoder present but fails to open -- on this "
                        "machine the driver exposes an older NVENC API than this ffmpeg needs); "
                        "using %s",
                        "h264_nvenc", c.label);
                }
                return *chosen;
            }
        }
        chosen = &kCandidates[3];   // libx264 is always the last word
        return *chosen;
    }
    FILE* m_f = nullptr;
    uint32_t m_w = 0, m_h = 0;
    bool m_reported = false;
};

}  // namespace ga::app
