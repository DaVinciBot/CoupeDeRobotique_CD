#pragma once
// Affichage via GStreamer (cv::imshow indisponible : OpenCV sans GTK/Qt sur la Jetson).
//
// Chaque mode essaie plusieurs pipelines dans l'ordre et garde le premier qui s'ouvre :
//  - UDP : encodeur matériel Jetson (nvv4l2h264enc, présent sur Jetson Nano / Xavier, ABSENT sur
//    Orin Nano) puis encodeur logiciel x264enc (tout Linux, Orin Nano compris).
//  - local : xvimagesink, ximagesink, puis autovideosink.
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
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

// Retourne un writer ouvert (ou non ouvert si mode None / aucun pipeline valide).
inline cv::VideoWriter makeDisplayWriter(DisplayMode mode, cv::Size size, double fps,
                                         const std::string& host, int port) {
    cv::VideoWriter w;
    if (mode == DisplayMode::None) return w;

    // label, pipeline
    std::vector<std::pair<std::string, std::string>> pipes;
    if (mode == DisplayMode::Local) {
        ensureXSession();
        for (const char* sink : {"xvimagesink", "ximagesink", "autovideosink"})
            pipes.emplace_back(sink, std::string("appsrc ! videoconvert ! ") + sink + " sync=false");
    } else {
        const std::string tail = " ! h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=" +
                                 host + " port=" + std::to_string(port) + " sync=false";
        // Encodeur matériel Jetson : l'image passe en mémoire NVMM via nvvidconv.
        pipes.emplace_back("nvv4l2h264enc (matériel)",
                           "appsrc ! videoconvert ! video/x-raw,format=BGRx ! nvvidconv ! "
                           "video/x-raw(memory:NVMM),format=I420 ! "
                           "nvv4l2h264enc bitrate=2000000 insert-sps-pps=true maxperf-enable=true" +
                               tail);
        pipes.emplace_back("x264enc (logiciel)",
                           "appsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast "
                           "bitrate=2000" + tail);
    }

    for (const auto& p : pipes) {
        if (w.open(p.second, cv::CAP_GSTREAMER, 0, fps, size, true)) {
            std::fprintf(stderr, "[display] %s\n", p.first.c_str());
            return w;
        }
    }
    std::fprintf(stderr, "[display] aucun pipeline GStreamer n'a pu s'ouvrir\n");
    return w;
}

}  // namespace utils
