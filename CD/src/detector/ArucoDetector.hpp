#pragma once
// Détection ArUco + projection dans le repère réel de l'arène (mètres) + yaw.
// Nécessite OpenCV >= 4.7 (cv::aruco::ArucoDetector dans le module objdetect).
#include <deque>
#include <map>
#include <set>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include "arena/ArenaElements.hpp"

namespace detector {

struct Detection {
    int id = -1;
    cv::Point2f pos;      // position monde (m)
    float yaw = 0.f;      // orientation monde (rad), 0 = axe x de l'arène
    bool carried = false; // true si reportée depuis une frame précédente (marqueur statique)
};

struct DetectorParams {
    bool useClahe = true;
    int warmupHits = 2;       // nb de détections requises ...
    int warmupWindow = 3;     // ... sur cette fenêtre de frames
    int refCacheFrames = 30;  // durée de vie d'une référence occultée
    int carryFrames = 10;     // durée de report des marqueurs statiques
    std::vector<int> kaplaIds;
    std::vector<double> scales{1.0, 0.6};  // fallback multi-échelle
};

class ArucoDetector {
public:
    ArucoDetector(const DetectorParams& params, const arena::ArenaElements& arena);

    // gray : image (GRAY8 ou BGR) déjà dé-distordue. Retourne les marqueurs hors références.
    std::vector<Detection> analyzeFrame(const cv::Mat& img);

    bool hasTransform() const { return haveH_; }
    int visibleRefCount() const { return visibleRefs_; }

    // Rendu 2D de l'arène avec les détections (BGR, 300 px/m).
    cv::Mat renderArena(const std::vector<Detection>& dets) const;

private:
    struct RawMarker {
        int id;
        cv::Point2f center;
        cv::Point2f dir;  // vecteur unitaire coin0 -> coin1 (image)
    };
    struct RefObs {
        cv::Point2f imgCenter;
        uint64_t frame = 0;
    };

    void applyClahe(const cv::Mat& in, cv::Mat& out);
    void detectMultiScale(const cv::Mat& img, std::map<int, RawMarker>& out);
    bool computeTransformFromRefs();
    void transformPointsToWorldBatch(const std::vector<cv::Point2f>& in,
                                     std::vector<cv::Point2f>& out) const;
    bool applyWarmupFilter(int id);
    bool isKapla(int id) const { return kaplaSet_.count(id) > 0; }
    bool isRef(int id) const { return arena_.refMarkers.count(id) > 0; }

    DetectorParams params_;
    arena::ArenaElements arena_;
    std::set<int> kaplaSet_;

    cv::aruco::ArucoDetector aruco_;
    cv::Ptr<cv::CLAHE> clahe_;

    uint64_t frameIdx_ = 0;
    std::map<int, RefObs> refCache_;
    cv::Mat H_;  // 3x3 CV_64F image -> monde
    bool haveH_ = false;
    int visibleRefs_ = 0;

    std::map<int, std::deque<uint64_t>> history_;
    struct LastSeen { Detection det; uint64_t frame; };
    std::map<int, LastSeen> lastSeen_;
};

}  // namespace detector
