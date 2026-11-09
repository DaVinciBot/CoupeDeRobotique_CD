#pragma once
// Capture CSI (IMX219) via GStreamer/nvargus, thread dédié qui ne garde que la dernière image.
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace camera {

struct CameraConfig {
    int sensorId = 0;
    int sensorMode = 0;
    int width = 1920;
    int height = 1080;
    int fps = 15;
    bool gray = true;
    std::string calibFile = "calibration.yml";
};

struct CameraStats {
    uint64_t framesCaptured = 0;
    uint64_t readFailures = 0;
    double fps = 0.0;
};

class CSICamera {
public:
    explicit CSICamera(const CameraConfig& cfg);
    ~CSICamera();

    bool open();
    void release();

    // Attend une image plus récente que lastId (renvoie false si timeout).
    bool readFrame(cv::Mat& out, uint64_t& lastId, int timeoutMs = 500);

    // Calibration : charge un fichier existant, ou calcule les cartes d'undistortion.
    bool loadCalibration(const std::string& path);
    bool initUndistortMaps();
    void undistortFrame(const cv::Mat& in, cv::Mat& out) const;
    bool hasCalibration() const { return !map1_.empty(); }

    // Collecte `views` vues d'un damier (coins intérieurs cols x rows), calibre et sauvegarde.
    bool calibrate(int cols, int rows, float squareM, int views, const std::atomic<bool>& stop);

    CameraStats getStats() const;
    std::string buildPipeline() const;

private:
    void captureLoop();

    CameraConfig cfg_;
    cv::VideoCapture cap_;
    std::thread thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex mtx_;
    std::condition_variable cv_;
    cv::Mat latest_;
    uint64_t frameId_ = 0;

    std::atomic<uint64_t> captured_{0};
    std::atomic<uint64_t> failures_{0};
    std::chrono::steady_clock::time_point t0_;

    cv::Mat K_, D_, map1_, map2_;
};

}  // namespace camera
