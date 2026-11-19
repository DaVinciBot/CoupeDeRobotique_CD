// Test MANUEL de la caméra Stereolabs ZED branchée en USB (nécessite la caméra et un écran).
//
// Principe : en USB, la ZED se présente comme une webcam UVC standard (/dev/videoX) qui renvoie les
// deux capteurs CÔTE À CÔTE dans une seule image (gauche | droite). On l'ouvre avec OpenCV (pas besoin
// du SDK ZED ni de CUDA) et on ne garde qu'UN seul des deux capteurs (gauche par défaut).
//
// Affichage : via GStreamer (comme src/utils/Display.hpp), car OpenCV est compilé sans GTK/Qt
// (cv::imshow indisponible). Les touches se tapent donc dans le TERMINAL, pas dans la fenêtre vidéo.
//
// Option : détection ArUco (activable/désactivable, voir ENABLE_ARUCO_CHECK ci-dessous). Quand un
// marqueur apparaît, un message est affiché dans le terminal et les marqueurs sont dessinés à l'écran.
//
// Ce test n'est PAS lancé par ctest (il faut le matériel). Lancement :
//   ./build/test_zed_camera                    # /dev/video0, 672x376 @ 30 i/s par capteur (réglage conseillé), ArUco selon le défaut
//   ./build/test_zed_camera --device 2         # autre périphérique
//   ./build/test_zed_camera --aruco            # force la détection ArUco
//   ./build/test_zed_camera --no-aruco         # désactive la détection ArUco
//   ./build/test_zed_camera --right            # utilise le capteur droit au lieu du gauche
//   ./build/test_zed_camera --res 1080         # 376 | 720 | 1080 | 2k (hauteur par capteur)
//   ./build/test_zed_camera --no-display       # sans fenêtre (messages ArUco dans le terminal seulement)
//
// Touches (dans le terminal) : 'a' active/désactive ArUco, 's' enregistre une capture, 'q' ou Ctrl+C quitte.
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/videoio.hpp>

#include "camera/ZedCamera.hpp"

namespace {

// ================================ RÉGLAGES RAPIDES ================================
// Mettre à false pour désactiver la détection ArUco par défaut (ou utiliser --aruco / --no-aruco,
// ou la touche 'a' pendant l'exécution).
constexpr bool ENABLE_ARUCO_CHECK = true;
// Même dictionnaire que le détecteur du projet (src/detector/ArucoDetector.cpp).
constexpr auto ARUCO_DICT = cv::aruco::DICT_4X4_100;
// Délai sans détection avant de considérer qu'un marqueur a disparu (évite les messages en rafale).
constexpr double LOST_AFTER_S = 1.0;
// ==================================================================================

struct Options {
    int device = 0;
    int height = 376;  // hauteur d'UN capteur (376 = meilleur compromis mesuré : 672x376 @ 30 i/s)
    int fps = 30;
    bool aruco = ENABLE_ARUCO_CHECK;
    bool useRight = false;
    bool display = true;
};

void usage(const char* prog) {
    std::printf("Usage : %s [--device N] [--res 376|720|1080|2k] [--fps N] [--aruco|--no-aruco] "
                "[--left|--right] [--no-display]\n",
                prog);
}

bool parseArgs(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
        if (a == "--device") {
            const char* v = next();
            if (!v) return false;
            o.device = std::atoi(v);
        } else if (a == "--res") {
            const char* v = next();
            if (!v) return false;
            const std::string s = v;
            o.height = (s == "2k" || s == "2K") ? 1242 : std::atoi(v);
            if (camera::zedSensorWidth(o.height) < 0) return false;
        } else if (a == "--fps") {
            const char* v = next();
            if (!v) return false;
            o.fps = std::atoi(v);
        } else if (a == "--aruco") {
            o.aruco = true;
        } else if (a == "--no-aruco") {
            o.aruco = false;
        } else if (a == "--left") {
            o.useRight = false;
        } else if (a == "--right") {
            o.useRight = true;
        } else if (a == "--no-display") {
            o.display = false;
        } else {
            return false;
        }
    }
    return true;
}

// --- Clavier dans le terminal (lecture d'une touche sans attendre Entrée) ---
volatile std::sig_atomic_t gStop = 0;
void onSignal(int) { gStop = 1; }

class RawTerminal {
public:
    RawTerminal() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &old_) != 0) return;
        termios raw = old_;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);  // touche par touche, sans écho
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        active_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
    }
    ~RawTerminal() {
        if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    }
    // Renvoie la touche appuyée, ou -1 si aucune.
    int poll() const {
        if (!active_) return -1;
        pollfd p{STDIN_FILENO, POLLIN, 0};
        if (::poll(&p, 1, 0) <= 0) return -1;
        unsigned char c = 0;
        return read(STDIN_FILENO, &c, 1) == 1 ? c : -1;
    }

private:
    termios old_{};
    bool active_ = false;
};

// --- Fenêtre d'affichage via GStreamer (essaie plusieurs sorties vidéo) ---
cv::VideoWriter openDisplay(cv::Size size, double fps) {
    if (!std::getenv("DISPLAY")) setenv("DISPLAY", ":0", 0);
    const char* sinks[] = {"ximagesink", "xvimagesink", "autovideosink"};
    for (const char* sink : sinks) {
        const std::string pipe = std::string("appsrc ! videoconvert ! ") + sink + " sync=false";
        cv::VideoWriter w;
        if (w.open(pipe, cv::CAP_GSTREAMER, 0, fps, size, true)) {
            std::printf("Affichage : GStreamer (%s)\n", sink);
            return w;
        }
    }
    return {};
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        usage(argv[0]);
        return 2;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // --- Ouverture de la caméra ---
    cv::VideoCapture cap(opt.device, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::fprintf(stderr, "ECHEC : impossible d'ouvrir /dev/video%d (caméra branchée ? droits ?)\n",
                     opt.device);
        return 1;
    }
    const int sensorW = camera::zedSensorWidth(opt.height);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 2 * sensorW);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, opt.height);
    cap.set(cv::CAP_PROP_FPS, opt.fps);

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
        std::fprintf(stderr, "ECHEC : la caméra est ouverte mais ne renvoie aucune image\n");
        return 1;
    }
    const double camFps = cap.get(cv::CAP_PROP_FPS);
    std::printf("Caméra ouverte : %dx%d @ %.0f fps (image stéréo complète)\n", frame.cols, frame.rows, camFps);
    if (frame.cols != 2 * sensorW || frame.rows != opt.height)
        std::printf("  Attention : résolution demandée %dx%d, obtenue %dx%d\n", 2 * sensorW, opt.height,
                    frame.cols, frame.rows);
    // Un rapport largeur/hauteur proche de 2 x 16/9 confirme une image côte à côte.
    const bool sideBySide = frame.cols >= 2 * frame.rows;
    if (!sideBySide)
        std::printf("  Attention : l'image ne semble pas côte à côte (est-ce bien une ZED ?), "
                    "l'image entière sera affichée\n");
    const cv::Size viewSize = sideBySide ? cv::Size(frame.cols / 2, frame.rows) : frame.size();

    // --- Affichage ---
    cv::VideoWriter display;
    if (opt.display) {
        display = openDisplay(viewSize, camFps > 0 ? camFps : opt.fps);
        if (!display.isOpened())
            std::fprintf(stderr, "  Attention : impossible d'ouvrir une fenêtre GStreamer, test sans "
                                 "affichage (vérifier DISPLAY et gstreamer1.0-x)\n");
    }

    std::printf("Capteur utilisé : %s | ArUco : %s\n", opt.useRight ? "droit" : "gauche",
                opt.aruco ? "activé" : "désactivé");
    std::printf("Touches (dans CE terminal) : 'a' ArUco on/off, 's' capture, 'q' ou Ctrl+C pour quitter\n");

    // --- Détecteur ArUco ---
    cv::aruco::ArucoDetector detector(cv::aruco::getPredefinedDictionary(ARUCO_DICT),
                                      cv::aruco::DetectorParameters());

    using Clock = std::chrono::steady_clock;
    std::map<int, Clock::time_point> lastSeen;  // id -> dernière détection
    RawTerminal term;

    int frames = 0, failedReads = 0, snapshots = 0;
    auto fpsStart = Clock::now();
    double fpsShown = 0.0;

    while (!gStop) {
        if (!cap.read(frame) || frame.empty()) {
            if (++failedReads > 30) {
                std::fprintf(stderr, "ECHEC : plus d'image reçue depuis 30 essais, arrêt\n");
                return 1;
            }
            continue;
        }
        failedReads = 0;

        // Une seule des deux caméras : moitié gauche ou droite de l'image.
        cv::Mat view = frame;
        if (sideBySide) {
            const int half = frame.cols / 2;
            view = frame(cv::Rect(opt.useRight ? half : 0, 0, half, frame.rows)).clone();
        }

        // --- Détection ArUco (optionnelle) ---
        if (opt.aruco) {
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            detector.detectMarkers(view, corners, ids);
            const auto now = Clock::now();

            for (int id : ids) {
                if (!lastSeen.count(id)) std::printf(">>> Marqueur ArUco DÉTECTÉ : id %d\n", id);
                lastSeen[id] = now;
            }
            for (auto it = lastSeen.begin(); it != lastSeen.end();) {
                if (std::chrono::duration<double>(now - it->second).count() > LOST_AFTER_S) {
                    std::printf("<<< Marqueur ArUco perdu : id %d\n", it->first);
                    it = lastSeen.erase(it);
                } else {
                    ++it;
                }
            }

            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(view, corners, ids);
                cv::putText(view, "ARUCO DETECTE (" + std::to_string(ids.size()) + ")", {20, 80},
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 0, 255}, 2);
            }
        }

        // --- Surimpression : fps + état ArUco ---
        ++frames;
        const double elapsed = std::chrono::duration<double>(Clock::now() - fpsStart).count();
        if (elapsed >= 1.0) {
            fpsShown = frames / elapsed;
            frames = 0;
            fpsStart = Clock::now();
        }
        char info[96];
        std::snprintf(info, sizeof(info), "%dx%d  %.1f fps  ArUco: %s", view.cols, view.rows, fpsShown,
                      opt.aruco ? "ON" : "OFF");
        cv::putText(view, info, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);

        if (display.isOpened()) display.write(view);

        // --- Clavier (terminal) ---
        const int key = term.poll();
        if (key == 'q' || key == 'Q' || key == 27) break;
        if (key == 'a' || key == 'A') {
            opt.aruco = !opt.aruco;
            lastSeen.clear();
            std::printf("ArUco %s\n", opt.aruco ? "activé" : "désactivé");
        }
        if (key == 's' || key == 'S') {
            const std::string name = "zed_capture_" + std::to_string(snapshots++) + ".png";
            cv::imwrite(name, view);
            std::printf("Capture enregistrée : %s\n", name.c_str());
        }
    }

    std::printf("Fin du test caméra ZED\n");
    return 0;
}
