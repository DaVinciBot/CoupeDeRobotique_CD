// Point d'entrée : charge la config, instancie les composants (caméra ZED en USB, détecteur ArUco,
// réseau) et fait tourner la boucle de traitement (PROCESS_FPS).
//
// Usage : remote_compute [--env chemin/.env]
// Clavier : taper une trame brute (ex. "1" ou "4|B") pour l'injecter comme message réseau entrant.
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "arena/ArenaElements.hpp"
#include "camera/ZedCamera.hpp"
#include "detector/ArucoDetector.hpp"
#include "game/MatchState.hpp"
#include "net/Protocol.hpp"
#include "net/WifiLink.hpp"
#include "utils/Config.hpp"
#include "utils/Display.hpp"
#include "utils/Timer.hpp"

using Clock = std::chrono::steady_clock;

static std::atomic<bool> g_stop{false};
static void onSignal(int) { g_stop = true; }

static void startKeyboardThread(net::WifiLink& link) {
    std::thread([&link] {
        std::string line;
        while (std::getline(std::cin, line)) {
            if (!line.empty()) link.injectRx(line);
        }
    }).detach();
}

static camera::CameraConfig loadCameraConfig(const utils::Config& c) {
    camera::CameraConfig cc;
    cc.device = c.integer("ZED_DEVICE", 0);
    cc.height = c.integer("ZED_RES", 376);
    cc.fps = c.integer("ZED_FPS", 30);
    cc.useRight = c.str("ZED_SIDE", "left") == "right";
    cc.gray = c.boolean("CAM_GRAY", true);
    return cc;
}

int main(int argc, char** argv) {
    std::string envPath = ".env";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--env" && i + 1 < argc) envPath = argv[++i];
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    utils::Config cfg;
    if (!cfg.load(envPath))
        std::fprintf(stderr, "[main] %s introuvable, valeurs par défaut / environnement\n", envPath.c_str());
    utils::setDebugMode(cfg.boolean("DEBUG", false));

    camera::ZedCamera cam(loadCameraConfig(cfg));
    if (!cam.open()) return 1;

    arena::ArenaElements arenaEl = arena::makeArena();

    detector::DetectorParams dp;
    dp.useClahe = cfg.boolean("USE_CLAHE", true);
    dp.warmupHits = cfg.integer("WARMUP_HITS", 2);
    dp.warmupWindow = cfg.integer("WARMUP_WINDOW", 3);
    dp.refCacheFrames = cfg.integer("REF_CACHE_FRAMES", 30);
    dp.carryFrames = cfg.integer("CARRY_FRAMES", 10);
    dp.kaplaIds = cfg.intList("KAPLA_IDS");
    detector::ArucoDetector det(dp, arenaEl);

    game::MatchState match;

    // ---------- Réseau Wi-Fi (UDP) ----------
    net::WifiLink link(cfg.integer("WIFI_LISTEN_PORT", 5005), cfg.integer("WIFI_MAX_TX_ERRORS", 5));
    if (!link.open()) {
        std::fprintf(stderr, "[main] impossible d'ouvrir le port UDP, arrêt\n");
        return 1;
    }
    {
        std::stringstream ss(cfg.str("WIFI_PEERS", ""));
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            if (tok.empty()) continue;
            if (!link.addPeer(tok)) std::fprintf(stderr, "[main] WIFI_PEERS : '%s' invalide (attendu ip:port)\n", tok.c_str());
        }
    }

    // cmd 1 : demande d'ID -> cmd 2 : ID attribué.
    // Clé stable = argument envoyé par le PAMI, sinon son adresse IP (un redémarrage garde le même ID).
    link.registerHandler(1, [&](const std::vector<std::string>& args, const net::Peer& from) {
        std::string key = !args.empty() ? args[0] : (from.valid() ? from.ip() : "");
        int id = match.registerPami(key);
        std::fprintf(stderr, "[main] PAMI enregistré, id=%d\n", id);
        link.reply(from, "2|" + std::to_string(id));
    });
    // cmd 4 : départ de match (B / Y)
    link.registerHandler(4, [&](const std::vector<std::string>& args, const net::Peer&) {
        char color = (!args.empty() && !args[0].empty()) ? args[0][0] : 'B';
        match.startMatch(color);
        std::fprintf(stderr, "[main] match démarré, couleur=%c\n", match.color());
        link.send(game::MatchState::buildMsg3(match.computeDepotAssignments(arenaEl)));
    });
    link.start();
    startKeyboardThread(link);

    // Affichage local optionnel de l'arène (GStreamer)
    auto mode = utils::parseDisplayMode(cfg.str("DISPLAY_MODE", "none"));
    cv::VideoWriter writer = utils::makeDisplayWriter(
        mode, cv::Size(static_cast<int>(arenaEl.width * 300), static_cast<int>(arenaEl.height * 300)),
        cfg.integer("PROCESS_FPS", 5));

    const auto procPeriod = std::chrono::milliseconds(1000 / std::max(1, cfg.integer("PROCESS_FPS", 5)));
    const auto visionPeriod = std::chrono::milliseconds(cfg.integer("VISION_SEND_PERIOD_MS", 500));
    const auto depotPeriod = std::chrono::milliseconds(cfg.integer("DEPOT_RESEND_PERIOD_MS", 3000));

    auto lastVision = Clock::now(), lastDepot = Clock::now(), lastStats = Clock::now();
    auto nextProc = Clock::now();
    uint64_t frameId = 0;
    cv::Mat frame;

    while (!g_stop) {
        std::this_thread::sleep_until(nextProc);
        nextProc = Clock::now() + procPeriod;

        if (!cam.readFrame(frame, frameId, 500)) continue;
        auto dets = det.analyzeFrame(frame);
        auto now = Clock::now();

        // Vision -> robot (cmd 5)
        if (now - lastVision >= visionPeriod) {
            lastVision = now;
            std::string msg = net::buildVisionMessage(dets);
            if (!msg.empty()) link.send(msg);
        }
        // Assignations de dépôt renvoyées périodiquement pendant le match (cmd 3)
        if (match.started() && !match.isOver() && now - lastDepot >= depotPeriod) {
            lastDepot = now;
            link.send(game::MatchState::buildMsg3(match.computeDepotAssignments(arenaEl)));
        }
        if (match.shouldSendPreEnd())
            std::fprintf(stderr, "[main] PRE_END : %.0f s restantes\n", game::MatchState::PRE_END_OFFSET);

        if (writer.isOpened()) writer.write(det.renderArena(dets));

        if (utils::debugFlag() && now - lastStats >= std::chrono::seconds(5)) {
            lastStats = now;
            auto st = cam.getStats();
            std::fprintf(stderr, "[main] cam %.1f fps, refs visibles=%d, détections=%zu, réseau=%s (%zu pairs)\n",
                         st.fps, det.visibleRefCount(), dets.size(), link.healthy() ? "ok" : "KO",
                         link.peerCount());
            utils::printTimerStats();
        }
    }

    std::fprintf(stderr, "[main] arrêt...\n");
    link.stop();
    cam.release();
    return 0;
}
