#include "detector/ArucoDetector.hpp"

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "utils/Timer.hpp"

namespace detector {

namespace {
cv::aruco::DetectorParameters makePermissiveParams() {
    cv::aruco::DetectorParameters p;
    // Sensibilité volontairement très permissive ; les faux positifs sont filtrés par le warm-up.
    p.adaptiveThreshWinSizeMin = 3;
    p.adaptiveThreshWinSizeMax = 53;
    p.adaptiveThreshWinSizeStep = 4;
    p.minMarkerPerimeterRate = 0.01;
    p.polygonalApproxAccuracyRate = 0.05;
    p.errorCorrectionRate = 0.8;
    p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    return p;
}
}  // namespace

ArucoDetector::ArucoDetector(const DetectorParams& params, const arena::ArenaElements& arena)
    : params_(params),
      arena_(arena),
      kaplaSet_(params.kaplaIds.begin(), params.kaplaIds.end()),
      aruco_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100),
             makePermissiveParams()) {
    clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));
    // TODO(jetson) : variante CUDA (cv::cuda::createCLAHE) si OpenCV est compilé avec cudaimgproc.
}

void ArucoDetector::applyClahe(const cv::Mat& in, cv::Mat& out) {
    cv::Mat g;
    if (in.channels() == 3) cv::cvtColor(in, g, cv::COLOR_BGR2GRAY);
    else g = in;
    if (params_.useClahe) clahe_->apply(g, out);
    else out = g;
}

void ArucoDetector::detectMultiScale(const cv::Mat& img, std::map<int, RawMarker>& out) {
    for (double s : params_.scales) {
        cv::Mat scaled;
        if (s == 1.0) scaled = img;
        else cv::resize(img, scaled, cv::Size(), s, s, cv::INTER_AREA);

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        aruco_.detectMarkers(scaled, corners, ids);

        for (size_t i = 0; i < ids.size(); ++i) {
            if (out.count(ids[i])) continue;  // déjà vu à une échelle précédente
            cv::Point2f c(0, 0);
            for (auto& p : corners[i]) c += p;
            c *= 0.25f / static_cast<float>(s);
            cv::Point2f d = (corners[i][1] - corners[i][0]);
            float n = std::hypot(d.x, d.y);
            if (n < 1e-6f) continue;
            out[ids[i]] = RawMarker{ids[i], c, d * (1.f / n)};
        }
    }
}

bool ArucoDetector::computeTransformFromRefs() {
    std::vector<cv::Point2f> src, dst;
    visibleRefs_ = 0;
    for (const auto& kv : refCache_) {
        uint64_t age = frameIdx_ - kv.second.frame;
        if (age == 0) visibleRefs_++;
        if (age > static_cast<uint64_t>(params_.refCacheFrames)) continue;
        auto it = arena_.refMarkers.find(kv.first);
        if (it == arena_.refMarkers.end()) continue;
        src.push_back(kv.second.imgCenter);
        dst.push_back(it->second);
    }
    if (src.size() >= 4) {
        cv::Mat H = cv::findHomography(src, dst, cv::RANSAC, 0.02);
        if (!H.empty()) { H.convertTo(H_, CV_64F); haveH_ = true; return true; }
    } else if (src.size() == 3) {
        cv::Mat A = cv::getAffineTransform(src.data(), dst.data());  // 2x3 CV_64F
        H_ = cv::Mat::eye(3, 3, CV_64F);
        A.copyTo(H_(cv::Rect(0, 0, 3, 2)));
        haveH_ = true;
        return true;
    }
    return haveH_;  // moins de 3 refs : on garde la dernière transformation connue
}

void ArucoDetector::transformPointsToWorldBatch(const std::vector<cv::Point2f>& in,
                                                std::vector<cv::Point2f>& out) const {
    out.clear();
    if (!haveH_ || in.empty()) return;
    cv::perspectiveTransform(in, out, H_);
}

bool ArucoDetector::applyWarmupFilter(int id) {
    auto& h = history_[id];
    h.push_back(frameIdx_);
    while (!h.empty() && frameIdx_ - h.front() >= static_cast<uint64_t>(params_.warmupWindow))
        h.pop_front();
    // Kaplas et références : codes d'un petit ensemble connu -> exemptés
    if (isKapla(id) || isRef(id)) return true;
    return static_cast<int>(h.size()) >= params_.warmupHits;
}

std::vector<Detection> ArucoDetector::analyzeFrame(const cv::Mat& img) {
    PROFILE_SCOPE("analyzeFrame");
    frameIdx_++;

    cv::Mat prepared;
    {
        PROFILE_SCOPE("clahe");
        applyClahe(img, prepared);
    }

    std::map<int, RawMarker> raw;
    {
        PROFILE_SCOPE("detectMarkers");
        detectMultiScale(prepared, raw);
    }

    // 1) mise à jour du cache des références + transformation image -> monde
    for (const auto& kv : raw)
        if (isRef(kv.first)) refCache_[kv.first] = RefObs{kv.second.center, frameIdx_};
    computeTransformFromRefs();

    std::vector<Detection> result;
    if (!haveH_) return result;

    // 2) projection batch : centre + point décalé le long du cap (pour le yaw)
    std::vector<int> ids;
    std::vector<cv::Point2f> pts;
    for (const auto& kv : raw) {
        if (isRef(kv.first)) continue;
        ids.push_back(kv.first);
        pts.push_back(kv.second.center);
        pts.push_back(kv.second.center + kv.second.dir * 20.f);
    }
    std::vector<cv::Point2f> world;
    {
        PROFILE_SCOPE("projectBatch");
        transformPointsToWorldBatch(pts, world);
    }

    std::set<int> seenNow;
    for (size_t i = 0; i < ids.size() && 2 * i + 1 < world.size(); ++i) {
        if (!applyWarmupFilter(ids[i])) continue;
        cv::Point2f a = world[2 * i], b = world[2 * i + 1];
        Detection d;
        d.id = ids[i];
        d.pos = a;
        d.yaw = std::atan2(b.y - a.y, b.x - a.x);
        result.push_back(d);
        seenNow.insert(d.id);
        lastSeen_[d.id] = LastSeen{d, frameIdx_};
    }
    // Les refs vues alimentent aussi l'historique (sans être renvoyées)
    for (const auto& kv : raw) if (isRef(kv.first)) applyWarmupFilter(kv.first);

    // 3) carry-forward des marqueurs statiques (kaplas) brièvement occultés
    for (const auto& kv : lastSeen_) {
        if (!isKapla(kv.first) || seenNow.count(kv.first)) continue;
        if (frameIdx_ - kv.second.frame <= static_cast<uint64_t>(params_.carryFrames)) {
            Detection d = kv.second.det;
            d.carried = true;
            result.push_back(d);
        }
    }
    return result;
}

cv::Mat ArucoDetector::renderArena(const std::vector<Detection>& dets) const {
    const float ppm = 300.f;
    cv::Mat img(static_cast<int>(arena_.height * ppm), static_cast<int>(arena_.width * ppm),
                CV_8UC3, cv::Scalar(40, 40, 40));
    auto toPx = [&](const cv::Point2f& p) { return cv::Point(int(p.x * ppm), int(p.y * ppm)); };
    auto drawZones = [&](const std::vector<arena::Zone>& zs) {
        for (const auto& z : zs) {
            cv::rectangle(img, toPx(z.origin),
                          toPx({z.origin.x + z.size.width, z.origin.y + z.size.height}), z.color, 2);
            cv::putText(img, z.name, toPx(z.origin) + cv::Point(4, 16),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, z.color, 1);
        }
    };
    drawZones(arena_.starting_zone);
    drawZones(arena_.zone_depot);
    drawZones(arena_.zone_ramassage);
    for (const auto& kv : arena_.refMarkers)
        cv::drawMarker(img, toPx(kv.second), {255, 255, 255}, cv::MARKER_CROSS, 14, 1);

    for (const auto& d : dets) {
        cv::Point c = toPx(d.pos);
        cv::Scalar col = d.carried ? cv::Scalar(150, 150, 150) : cv::Scalar(0, 255, 0);
        cv::circle(img, c, 8, col, -1);
        cv::line(img, c, c + cv::Point(int(20 * std::cos(d.yaw)), int(20 * std::sin(d.yaw))), col, 2);
        cv::putText(img, std::to_string(d.id), c + cv::Point(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1);
    }
    return img;
}

}  // namespace detector
