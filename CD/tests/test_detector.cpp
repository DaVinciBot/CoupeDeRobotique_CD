// Tests du détecteur ArUco sur des images SYNTHÉTIQUES (aucune caméra nécessaire).
//
// Principe : on dessine une vue de dessus de l'arène (600 px par mètre) avec les marqueurs de
// référence de ArenaElements et des marqueurs "kapla" à des positions connues. Le détecteur doit
// retrouver ces positions (en mètres) et ces orientations. On dégrade ensuite l'image (perspective,
// bruit, flou, ombre) pour vérifier la robustesse.
//
// Limite : ces tests ne remplacent pas un essai avec la vraie caméra (distorsion d'objectif,
// éclairage réel, flou de mouvement).
//
// Convention vérifiée : yaw = 0 quand le bord haut du marqueur pointe vers +x ; l'axe y de l'arène
// va vers le bas, donc une rotation visuellement anti-horaire de +30° donne un yaw de -30°.
#include <cmath>
#include <memory>
#include <set>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include "check.hpp"
#include "detector/ArucoDetector.hpp"

namespace {

constexpr float kPpm = 600.f;  // pixels par mètre dans l'image synthétique
const arena::ArenaElements kArena = arena::makeArena();

struct Item {
    int id;
    cv::Point2f pos;    // mètres
    double angleDeg;    // rotation dessinée (cv::getRotationMatrix2D : positif = anti-horaire à l'écran)
};

void place(cv::Mat& canvas, const Item& it, int sizePx = 70) {
    cv::Mat m;
    cv::aruco::generateImageMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100), it.id,
                                   sizePx, m, 1);
    cv::copyMakeBorder(m, m, 12, 12, 12, 12, cv::BORDER_CONSTANT, 255);  // marge blanche autour
    cv::Mat rot = cv::getRotationMatrix2D({m.cols / 2.f, m.rows / 2.f}, it.angleDeg, 1.0);
    rot.at<double>(0, 2) += it.pos.x * kPpm - m.cols / 2.f;
    rot.at<double>(1, 2) += it.pos.y * kPpm - m.rows / 2.f;
    cv::warpAffine(m, canvas, rot, canvas.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
}

// Scène : toutes les références (sauf celles masquées) + les marqueurs demandés.
cv::Mat makeScene(const std::vector<Item>& items, const std::set<int>& hiddenRefs = {}) {
    cv::Mat img(static_cast<int>(kArena.height * kPpm), static_cast<int>(kArena.width * kPpm), CV_8UC1,
                cv::Scalar(255));
    for (const auto& kv : kArena.refMarkers)
        if (!hiddenRefs.count(kv.first)) place(img, {kv.first, kv.second, 0.0});
    for (const auto& it : items) place(img, it);
    return img;
}

std::unique_ptr<detector::ArucoDetector> makeDetector() {
    detector::DetectorParams dp;
    dp.kaplaIds = {36, 37, 38, 39};
    return std::make_unique<detector::ArucoDetector>(dp, kArena);
}

const detector::Detection* find(const std::vector<detector::Detection>& v, int id) {
    for (const auto& d : v)
        if (d.id == id) return &d;
    return nullptr;
}

// Écart absolu entre deux angles en degrés, modulo 360.
double angDiff(double a, double b) { return std::fabs(std::fmod(a - b + 540.0, 360.0) - 180.0); }

std::vector<detector::Detection> run(detector::ArucoDetector& det, const cv::Mat& img, int frames = 3) {
    std::vector<detector::Detection> out;
    for (int i = 0; i < frames; ++i) out = det.analyzeFrame(img);
    return out;
}

float yawDeg(const detector::Detection& d) { return d.yaw * 180.f / 3.14159265f; }

// ------------------------------------------------------------------------------------------------

void testPositionAndYaw() {
    auto det = makeDetector();
    for (double angle : {0.0, 30.0, -60.0, 90.0, 135.0, 180.0, -135.0}) {
        auto out = run(*det, makeScene({{36, {1.50f, 1.00f}, angle}}));
        const auto* d = find(out, 36);
        CHECK(d != nullptr);
        if (!d) continue;
        CHECK_NEAR(d->pos.x, 1.50, 0.01);
        CHECK_NEAR(d->pos.y, 1.00, 0.01);
        CHECK(angDiff(yawDeg(*d), -angle) < 3.0);  // convention : voir l'en-tête du fichier
        CHECK(!d->carried);
    }
}

void testPositionsAcrossArena() {
    auto det = makeDetector();
    for (float x : {0.30f, 1.00f, 1.50f, 2.00f, 2.70f})
        for (float y : {0.30f, 1.00f, 1.70f}) {
            auto out = run(*det, makeScene({{36, {x, y}, 20.0}}), 2);
            const auto* d = find(out, 36);
            CHECK(d != nullptr);
            if (!d) continue;
            CHECK_NEAR(d->pos.x, x, 0.01);
            CHECK_NEAR(d->pos.y, y, 0.01);
        }
}

void testSeveralMarkersKeepTheirIds() {
    auto det = makeDetector();
    std::vector<Item> items = {{36, {0.30f, 0.30f}, 0}, {37, {1.50f, 1.00f}, 45}, {38, {2.70f, 1.70f}, -90},
                               {39, {1.20f, 0.30f}, 10}};
    auto out = run(*det, makeScene(items));
    for (const auto& it : items) {
        const auto* d = find(out, it.id);
        CHECK(d != nullptr);
        if (!d) continue;
        CHECK_NEAR(d->pos.x, it.pos.x, 0.01);  // l'id 36 ne doit pas se retrouver à la position du 37...
        CHECK_NEAR(d->pos.y, it.pos.y, 0.01);
    }
}

void testReferencesAreNotReturned() {
    auto det = makeDetector();
    auto out = run(*det, makeScene({{36, {1.5f, 1.0f}, 0}}));
    for (const auto& kv : kArena.refMarkers) CHECK(find(out, kv.first) == nullptr);
    CHECK(det->hasTransform());
    CHECK_EQ(det->visibleRefCount(), 4);
}

void testWarmupFiltersFalsePositives() {
    const Item unknown{5, {1.5f, 1.0f}, 0};  // ni référence ni kapla : soumis au warm-up (2 vues sur 3)
    const cv::Mat with = makeScene({unknown}), without = makeScene({});

    {  // vu 2 images de suite : accepté à la 2e
        auto det = makeDetector();
        CHECK(find(det->analyzeFrame(with), 5) == nullptr);
        CHECK(find(det->analyzeFrame(with), 5) != nullptr);
    }
    {  // vu / absent / vu : 2 vues sur 3 -> accepté à la 3e
        auto det = makeDetector();
        det->analyzeFrame(with);
        CHECK(find(det->analyzeFrame(without), 5) == nullptr);
        CHECK(find(det->analyzeFrame(with), 5) != nullptr);
    }
    {  // vu / absent / absent / vu : la 1re vue est sortie de la fenêtre -> refusé
        auto det = makeDetector();
        det->analyzeFrame(with);
        det->analyzeFrame(without);
        det->analyzeFrame(without);
        CHECK(find(det->analyzeFrame(with), 5) == nullptr);
    }
}

void testKaplaCarryForward() {
    auto det = makeDetector();
    const cv::Mat with = makeScene({{36, {1.5f, 1.0f}, 0}}), without = makeScene({});
    det->analyzeFrame(with);
    det->analyzeFrame(with);
    // carryFrames = 10 par défaut : reporté (carried) pendant 10 images, puis oublié
    for (int k = 1; k <= 10; ++k) {
        const auto out = det->analyzeFrame(without);  // garder le vecteur : find() renvoie un pointeur dedans
        const auto* d = find(out, 36);
        CHECK(d != nullptr);
        if (d) {
            CHECK(d->carried);
            CHECK_NEAR(d->pos.x, 1.5, 0.01);
        }
    }
    CHECK(find(det->analyzeFrame(without), 36) == nullptr);
    // et revient normalement quand il réapparaît
    const auto outBack = det->analyzeFrame(with);
    const auto* back = find(outBack, 36);
    CHECK(back != nullptr);
    if (back) CHECK(!back->carried);
}

void testOccludedReferenceStaysUsable() {
    auto det = makeDetector();
    const std::vector<Item> kapla = {{36, {1.5f, 1.0f}, 25}};
    run(*det, makeScene(kapla));
    // Une référence disparaît : le cache (30 images) permet de garder la même précision.
    for (int i = 0; i < 20; ++i) {
        auto out = det->analyzeFrame(makeScene(kapla, {23}));
        const auto* d = find(out, 36);
        CHECK(d != nullptr);
        if (d) {
            CHECK_NEAR(d->pos.x, 1.5, 0.01);
            CHECK_NEAR(d->pos.y, 1.0, 0.01);
        }
    }
    CHECK_EQ(det->visibleRefCount(), 3);
    // Après expiration du cache : 3 références seulement -> transformation affine, toujours correcte.
    std::vector<detector::Detection> out;
    for (int i = 0; i < 25; ++i) out = det->analyzeFrame(makeScene(kapla, {23}));
    const auto* d = find(out, 36);
    CHECK(d != nullptr);
    if (d) {
        CHECK_NEAR(d->pos.x, 1.5, 0.01);
        CHECK_NEAR(d->pos.y, 1.0, 0.01);
    }
}

void testNotEnoughReferences() {
    {  // 3 références : OK (affine)
        auto det = makeDetector();
        auto out = run(*det, makeScene({{36, {1.5f, 1.0f}, 0}}, {22}));
        const auto* d = find(out, 36);
        CHECK(d != nullptr);
        if (d) CHECK_NEAR(d->pos.x, 1.5, 0.01);
    }
    {  // 2 références : impossible de passer en mètres -> aucune sortie plutôt qu'une valeur fausse
        auto det = makeDetector();
        auto out = run(*det, makeScene({{36, {1.5f, 1.0f}, 0}}, {22, 23}));
        CHECK(!det->hasTransform());
        CHECK(out.empty());
    }
}

void testPerspectiveCamera() {
    // Caméra non parfaitement au-dessus de la table : l'image est déformée en perspective,
    // mais les références (qui subissent la même déformation) doivent permettre de tout retrouver.
    const cv::Mat scene = makeScene({{36, {1.5f, 1.0f}, 30}, {37, {0.8f, 1.6f}, -20}});
    const float w = static_cast<float>(scene.cols), h = static_cast<float>(scene.rows);
    const cv::Point2f src[4] = {{0, 0}, {w, 0}, {w, h}, {0, h}};
    const cv::Point2f dst[4] = {{70, 40}, {w - 30, 10}, {w - 90, h - 50}, {20, h - 20}};
    cv::Mat warped;
    cv::warpPerspective(scene, warped, cv::getPerspectiveTransform(src, dst), scene.size(), cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT, cv::Scalar(255));

    auto det = makeDetector();
    auto out = run(*det, warped);
    const auto* a = find(out, 36);
    const auto* b = find(out, 37);
    CHECK(a != nullptr);
    CHECK(b != nullptr);
    if (a) {
        CHECK_NEAR(a->pos.x, 1.5, 0.02);
        CHECK_NEAR(a->pos.y, 1.0, 0.02);
        CHECK(angDiff(yawDeg(*a), -30.0) < 6.0);
    }
    if (b) {
        CHECK_NEAR(b->pos.x, 0.8, 0.02);
        CHECK_NEAR(b->pos.y, 1.6, 0.02);
    }
}

void testNoiseBlurAndShadow() {
    cv::Mat img = makeScene({{36, {1.5f, 1.0f}, 30}});
    cv::GaussianBlur(img, img, cv::Size(3, 3), 1.0);

    cv::Mat f;
    img.convertTo(f, CV_32F);
    for (int y = 0; y < f.rows; ++y)  // ombre : luminosité de 60 % (gauche) à 100 % (droite)
        for (int x = 0; x < f.cols; ++x) f.at<float>(y, x) *= 0.6f + 0.4f * static_cast<float>(x) / f.cols;

    cv::RNG rng(1234);  // graine fixe : test reproductible
    cv::Mat noise(f.size(), CV_32F);
    rng.fill(noise, cv::RNG::NORMAL, 0.0, 10.0);
    f += noise;
    f.convertTo(img, CV_8U);

    auto det = makeDetector();
    auto out = run(*det, img);
    const auto* d = find(out, 36);
    CHECK(d != nullptr);
    if (d) {
        CHECK_NEAR(d->pos.x, 1.5, 0.015);
        CHECK_NEAR(d->pos.y, 1.0, 0.015);
        CHECK(angDiff(yawDeg(*d), -30.0) < 5.0);
    }
}

void testEmptyImage() {
    auto det = makeDetector();
    cv::Mat blank(1200, 1800, CV_8UC1, cv::Scalar(255));
    auto out = run(*det, blank);
    CHECK(out.empty());
    CHECK(!det->hasTransform());
}

}  // namespace

int main() {
    RUN(testPositionAndYaw);
    RUN(testPositionsAcrossArena);
    RUN(testSeveralMarkersKeepTheirIds);
    RUN(testReferencesAreNotReturned);
    RUN(testWarmupFiltersFalsePositives);
    RUN(testKaplaCarryForward);
    RUN(testOccludedReferenceStaysUsable);
    RUN(testNotEnoughReferences);
    RUN(testPerspectiveCamera);
    RUN(testNoiseBlurAndShadow);
    RUN(testEmptyImage);
    return testing_util::finish("test_detector");
}
