#include "camera/CSICamera.hpp"

#include <chrono>
#include <cstdio>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace camera {

using Clock = std::chrono::steady_clock;

CSICamera::CSICamera(const CameraConfig& cfg) : cfg_(cfg) {}

CSICamera::~CSICamera() { release(); }

std::string CSICamera::buildPipeline() const {
    std::string p =
        "nvarguscamerasrc sensor-id=" + std::to_string(cfg_.sensorId) +
        " sensor-mode=" + std::to_string(cfg_.sensorMode) +
        " ! video/x-raw(memory:NVMM),width=" + std::to_string(cfg_.width) +
        ",height=" + std::to_string(cfg_.height) +
        ",framerate=" + std::to_string(cfg_.fps) + "/1"
        " ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! ";
    p += cfg_.gray ? "video/x-raw,format=GRAY8" : "video/x-raw,format=BGR";
    // drop=true + max-buffers=1 : on ne garde que la dernière image (latence minimale)
    p += " ! appsink drop=true max-buffers=1 sync=false";
    return p;
}

bool CSICamera::open() {
    if (running_) return true;
    const std::string pipe = buildPipeline();
    std::fprintf(stderr, "[camera] pipeline: %s\n", pipe.c_str());
    cap_.open(pipe, cv::CAP_GSTREAMER);
    if (!cap_.isOpened()) {
        std::fprintf(stderr, "[camera] échec d'ouverture (OpenCV compilé avec GStreamer ? nvargus actif ?)\n");
        return false;
    }
    t0_ = Clock::now();
    running_ = true;
    thread_ = std::thread(&CSICamera::captureLoop, this);
    return true;
}

void CSICamera::release() {
    if (!running_.exchange(false)) {
        if (cap_.isOpened()) cap_.release();
        return;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    cap_.release();
}

void CSICamera::captureLoop() {
    cv::Mat frame;
    while (running_) {
        if (!cap_.read(frame) || frame.empty()) {
            failures_++;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        {
            std::lock_guard<std::mutex> lk(mtx_);
            frame.copyTo(latest_);
            frameId_++;
        }
        captured_++;
        cv_.notify_all();
    }
}

bool CSICamera::readFrame(cv::Mat& out, uint64_t& lastId, int timeoutMs) {
    std::unique_lock<std::mutex> lk(mtx_);
    cv_.wait_for(lk, std::chrono::milliseconds(timeoutMs),
                 [&] { return frameId_ > lastId || !running_; });
    if (frameId_ <= lastId || latest_.empty()) return false;
    latest_.copyTo(out);
    lastId = frameId_;
    return true;
}

bool CSICamera::loadCalibration(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["camera_matrix"] >> K_;
    fs["dist_coeffs"] >> D_;
    if (K_.empty() || D_.empty()) return false;
    return true;
}

bool CSICamera::initUndistortMaps() {
    if (K_.empty() || D_.empty()) return false;
    cv::Size size(cfg_.width, cfg_.height);
    cv::Mat newK = cv::getOptimalNewCameraMatrix(K_, D_, size, 0.0, size);
    cv::initUndistortRectifyMap(K_, D_, cv::Mat(), newK, size, CV_16SC2, map1_, map2_);
    return true;
}

void CSICamera::undistortFrame(const cv::Mat& in, cv::Mat& out) const {
    if (map1_.empty()) { in.copyTo(out); return; }
    cv::remap(in, out, map1_, map2_, cv::INTER_LINEAR);
}

bool CSICamera::calibrate(int cols, int rows, float squareM, int views,
                          const std::atomic<bool>& stop) {
    const cv::Size pattern(cols, rows);
    std::vector<cv::Point3f> objp;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) objp.emplace_back(c * squareM, r * squareM, 0.f);

    std::vector<std::vector<cv::Point3f>> objPts;
    std::vector<std::vector<cv::Point2f>> imgPts;
    auto last = Clock::now() - std::chrono::seconds(5);
    uint64_t id = 0;
    cv::Mat frame, gray;
    std::fprintf(stderr, "[calib] présentez le damier %dx%d sous différents angles (%d vues)\n",
                 cols, rows, views);

    while (!stop && (int)objPts.size() < views) {
        if (!readFrame(frame, id, 500)) continue;
        if (frame.channels() == 3) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        else gray = frame;

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray, pattern, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        double since = std::chrono::duration<double>(Clock::now() - last).count();
        if (!found || since < 1.0) continue;

        cv::cornerSubPix(gray, corners, {11, 11}, {-1, -1},
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
        objPts.push_back(objp);
        imgPts.push_back(corners);
        last = Clock::now();
        std::fprintf(stderr, "[calib] vue %zu/%d\n", objPts.size(), views);
    }
    if ((int)objPts.size() < 5) {
        std::fprintf(stderr, "[calib] pas assez de vues, abandon\n");
        return false;
    }

    std::vector<cv::Mat> rvecs, tvecs;
    cv::Size size(gray.cols, gray.rows);
    double rms = cv::calibrateCamera(objPts, imgPts, size, K_, D_, rvecs, tvecs);
    std::fprintf(stderr, "[calib] erreur RMS de reprojection: %.3f px\n", rms);

    cv::FileStorage fs(cfg_.calibFile, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    fs << "image_width" << size.width << "image_height" << size.height
       << "camera_matrix" << K_ << "dist_coeffs" << D_ << "rms" << rms;
    std::fprintf(stderr, "[calib] sauvegardé dans %s\n", cfg_.calibFile.c_str());
    return true;
}

CameraStats CSICamera::getStats() const {
    CameraStats s;
    s.framesCaptured = captured_;
    s.readFailures = failures_;
    double dt = std::chrono::duration<double>(Clock::now() - t0_).count();
    s.fps = dt > 0 ? s.framesCaptured / dt : 0.0;
    return s;
}

}  // namespace camera
