#pragma once
// Affichage via GStreamer (cv::imshow indisponible : OpenCV sans GTK/Qt sur la Jetson).
#include <cstdlib>
#include <string>
#include <opencv2/videoio.hpp>

namespace utils {

enum class DisplayMode { None, Local, Udp };

inline DisplayMode parseDisplayMode(const std::string& s) {
    if (s == "local") return DisplayMode::Local;
    if (s == "udp") return DisplayMode::Udp;
    return DisplayMode::None;
}

inline void ensureXSession() {
    if (!std::getenv("DISPLAY")) setenv("DISPLAY", ":0", 0);
}

// Retourne un writer ouvert (ou non ouvert si mode None / pipeline invalide).
inline cv::VideoWriter makeDisplayWriter(DisplayMode mode, cv::Size size, double fps,
                                         const std::string& host, int port) {
    cv::VideoWriter w;
    if (mode == DisplayMode::None) return w;
    std::string pipe;
    if (mode == DisplayMode::Local) {
        ensureXSession();
        pipe = "appsrc ! videoconvert ! xvimagesink sync=false";
    } else {
        // Sur Jetson, nvv4l2h264enc (matériel) peut remplacer x264enc.
        pipe = "appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast "
               "bitrate=2000 ! h264parse ! rtph264pay config-interval=1 pt=96 ! "
               "udpsink host=" + host + " port=" + std::to_string(port) + " sync=false";
    }
    w.open(pipe, cv::CAP_GSTREAMER, 0, fps, size, true);
    return w;
}

}  // namespace utils
