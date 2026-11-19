#pragma once
// Affichage local via GStreamer (cv::imshow indisponible : OpenCV sans GTK/Qt sur la Jetson).
// Essaie plusieurs sorties vidéo et garde la première qui s'ouvre.
// (L'affichage à distance par UDP a été retiré pour l'instant.)
#include <cstdio>
#include <cstdlib>
#include <string>
#include <opencv2/videoio.hpp>

namespace utils {

enum class DisplayMode { None, Local };

inline DisplayMode parseDisplayMode(const std::string& s) {
    return s == "local" ? DisplayMode::Local : DisplayMode::None;
}

inline void ensureXSession() {
    if (!std::getenv("DISPLAY")) setenv("DISPLAY", ":0", 0);
}

// Retourne un writer ouvert (ou non ouvert si mode None / aucune sortie vidéo disponible).
inline cv::VideoWriter makeDisplayWriter(DisplayMode mode, cv::Size size, double fps) {
    cv::VideoWriter w;
    if (mode == DisplayMode::None) return w;
    ensureXSession();
    for (const char* sink : {"xvimagesink", "ximagesink", "autovideosink"}) {
        const std::string pipe = std::string("appsrc ! videoconvert ! ") + sink + " sync=false";
        if (w.open(pipe, cv::CAP_GSTREAMER, 0, fps, size, true)) {
            std::fprintf(stderr, "[display] %s\n", sink);
            return w;
        }
    }
    std::fprintf(stderr, "[display] aucune sortie vidéo GStreamer n'a pu s'ouvrir\n");
    return w;
}

}  // namespace utils
