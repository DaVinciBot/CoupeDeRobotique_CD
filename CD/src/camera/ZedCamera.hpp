#pragma once
// Capture de la caméra Stereolabs ZED branchée en USB, thread dédié qui ne garde que la dernière image.
//
// En USB, la ZED est une webcam UVC standard (/dev/videoX) : pas besoin du SDK ZED ni de CUDA.
// Elle renvoie les deux capteurs CÔTE À CÔTE dans une seule image (gauche | droite) ; on ne garde
// qu'un seul capteur (gauche par défaut).
//
// Réglage conseillé (meilleur compromis mesuré) : 672x376 par capteur à 30 i/s.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace camera {

// Modes vidéo de la ZED en UVC : largeur d'UN capteur pour une hauteur donnée (-1 si inconnue).
// La largeur de l'image complète est le double (deux capteurs côte à côte).
inline int zedSensorWidth(int height) {
    switch (height) {
        case 376: return 672;
        case 720: return 1280;
        case 1080: return 1920;
        case 1242: return 2208;  // "2K"
        default: return -1;
    }
}

struct CameraConfig {
    int device = 0;      // /dev/videoN
    int height = 376;    // hauteur d'UN capteur : 376 | 720 | 1080 | 1242
    int fps = 30;
    bool useRight = false;  // capteur droit au lieu du gauche
    bool gray = true;       // convertir en niveaux de gris (ce qu'utilise le détecteur)
};

struct CameraStats {
    uint64_t framesCaptured = 0;
    uint64_t readFailures = 0;
    double fps = 0.0;
};

class ZedCamera {
public:
    explicit ZedCamera(const CameraConfig& cfg);
    ~ZedCamera();

    bool open();
    void release();

    // Attend une image plus récente que lastId (renvoie false si timeout).
    bool readFrame(cv::Mat& out, uint64_t& lastId, int timeoutMs = 500);

    CameraStats getStats() const;
    cv::Size frameSize() const { return size_; }  // taille d'un capteur, connue après open()

private:
    void captureLoop();
    void extractView(const cv::Mat& full, cv::Mat& out) const;

    CameraConfig cfg_;
    cv::VideoCapture cap_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    bool sideBySide_ = true;
    cv::Size size_;

    mutable std::mutex mtx_;
    std::condition_variable cv_;
    cv::Mat latest_;
    uint64_t frameId_ = 0;

    std::atomic<uint64_t> captured_{0};
    std::atomic<uint64_t> failures_{0};
    std::chrono::steady_clock::time_point t0_;
};

}  // namespace camera
