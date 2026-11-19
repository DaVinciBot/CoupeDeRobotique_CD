#include "camera/ZedCamera.hpp"

#include <cstdio>
#include <opencv2/imgproc.hpp>

namespace camera {

using Clock = std::chrono::steady_clock;

ZedCamera::ZedCamera(const CameraConfig& cfg) : cfg_(cfg) {}

ZedCamera::~ZedCamera() { release(); }

bool ZedCamera::open() {
    if (running_) return true;
    const int sensorW = zedSensorWidth(cfg_.height);
    if (sensorW < 0) {
        std::fprintf(stderr, "[camera] résolution ZED inconnue : %d (attendu 376, 720, 1080 ou 1242)\n",
                     cfg_.height);
        return false;
    }

    cap_.open(cfg_.device, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        std::fprintf(stderr, "[camera] impossible d'ouvrir /dev/video%d (ZED branchée ? droits ?)\n",
                     cfg_.device);
        return false;
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 2 * sensorW);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
    cap_.set(cv::CAP_PROP_FPS, cfg_.fps);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);  // latence minimale (ignoré si le pilote ne le gère pas)

    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
        std::fprintf(stderr, "[camera] la caméra est ouverte mais ne renvoie aucune image\n");
        cap_.release();
        return false;
    }
    if (frame.cols != 2 * sensorW || frame.rows != cfg_.height)
        std::fprintf(stderr, "[camera] attention : demandé %dx%d, obtenu %dx%d\n", 2 * sensorW,
                     cfg_.height, frame.cols, frame.rows);
    // Une image au moins deux fois plus large que haute = deux capteurs côte à côte.
    sideBySide_ = frame.cols >= 2 * frame.rows;
    if (!sideBySide_)
        std::fprintf(stderr, "[camera] attention : image non côte à côte (est-ce bien une ZED ?), "
                             "image entière utilisée\n");
    size_ = sideBySide_ ? cv::Size(frame.cols / 2, frame.rows) : frame.size();
    std::fprintf(stderr, "[camera] ZED /dev/video%d : capteur %s %dx%d @ %.0f i/s\n", cfg_.device,
                 cfg_.useRight ? "droit" : "gauche", size_.width, size_.height, cap_.get(cv::CAP_PROP_FPS));

    t0_ = Clock::now();
    running_ = true;
    thread_ = std::thread(&ZedCamera::captureLoop, this);
    return true;
}

void ZedCamera::release() {
    if (!running_.exchange(false)) {
        if (cap_.isOpened()) cap_.release();
        return;
    }
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    cap_.release();
}

void ZedCamera::extractView(const cv::Mat& full, cv::Mat& out) const {
    cv::Mat view = full;
    if (sideBySide_) {
        const int half = full.cols / 2;
        view = full(cv::Rect(cfg_.useRight ? half : 0, 0, half, full.rows));
    }
    if (cfg_.gray && view.channels() == 3) cv::cvtColor(view, out, cv::COLOR_BGR2GRAY);
    else view.copyTo(out);
}

void ZedCamera::captureLoop() {
    cv::Mat frame, view;
    while (running_) {
        if (!cap_.read(frame) || frame.empty()) {
            failures_++;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        extractView(frame, view);
        {
            std::lock_guard<std::mutex> lk(mtx_);
            view.copyTo(latest_);
            frameId_++;
        }
        captured_++;
        cv_.notify_all();
    }
}

bool ZedCamera::readFrame(cv::Mat& out, uint64_t& lastId, int timeoutMs) {
    std::unique_lock<std::mutex> lk(mtx_);
    cv_.wait_for(lk, std::chrono::milliseconds(timeoutMs),
                 [&] { return frameId_ > lastId || !running_; });
    if (frameId_ <= lastId || latest_.empty()) return false;
    latest_.copyTo(out);
    lastId = frameId_;
    return true;
}

CameraStats ZedCamera::getStats() const {
    CameraStats s;
    s.framesCaptured = captured_;
    s.readFailures = failures_;
    double dt = std::chrono::duration<double>(Clock::now() - t0_).count();
    s.fps = dt > 0 ? s.framesCaptured / dt : 0.0;
    return s;
}

}  // namespace camera
