#pragma once
// Format des messages échangés entre la Jetson et les robots (trames texte "cmd|arg|arg").
// Construction ET décodage au même endroit : ce fichier sert de référence du protocole,
// et les tests (tests/test_protocol.cpp) vérifient que les deux restent cohérents.
//
//   cmd 5 (Jetson -> robot)  : "5|id|x_cm|y_cm|yaw_deg|id|x_cm|y_cm|yaw_deg|..."
//        x, y : position en centimètres dans le repère de l'arène (origine en haut à gauche)
//        yaw  : orientation en degrés entiers, [-180, 180], 0 = axe x de l'arène
//   cmd 3 (Jetson -> PAMIs)  : "3|pami|zone|pami|zone|..."   (MatchState::buildMsg3)
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include "detector/ArucoDetector.hpp"
#include "game/MatchState.hpp"

namespace net {

constexpr size_t kMaxVisionItems = 40;

struct VisionItem {
    int id = 0;
    int x_cm = 0;
    int y_cm = 0;
    int yaw_deg = 0;
};

// Retourne "" s'il n'y a aucune détection (rien à envoyer).
inline std::string buildVisionMessage(const std::vector<detector::Detection>& dets,
                                      size_t maxItems = kMaxVisionItems) {
    if (dets.empty()) return "";
    constexpr float kPi = 3.14159265358979323846f;
    std::string s = "5";
    size_t n = 0;
    for (const auto& d : dets) {
        if (n++ >= maxItems) break;
        int x = static_cast<int>(std::lround(d.pos.x * 100.f));
        int y = static_cast<int>(std::lround(d.pos.y * 100.f));
        int yaw = static_cast<int>(std::lround(d.yaw * 180.f / kPi));
        s += "|" + std::to_string(d.id) + "|" + std::to_string(x) + "|" + std::to_string(y) + "|" +
             std::to_string(yaw);
    }
    return s;
}

inline std::vector<std::string> splitFrame(const std::string& frame) {
    std::vector<std::string> parts;
    std::stringstream ss(frame);
    std::string tok;
    while (std::getline(ss, tok, '|')) parts.push_back(tok);
    return parts;
}

// Entier strict : toute la chaîne doit être un entier (signe autorisé), pas de texte parasite.
inline bool parseInt(const std::string& s, int& out) {
    if (s.empty()) return false;
    char* end = nullptr;
    long v = std::strtol(s.c_str(), &end, 10);
    if (*end != '\0') return false;
    out = static_cast<int>(v);
    return true;
}

// Décode une trame cmd 5. false si la trame est invalide (autre commande, nombre d'arguments
// qui n'est pas un multiple de 4, valeur non numérique, aucun marqueur).
inline bool parseVisionMessage(const std::string& frame, std::vector<VisionItem>& out) {
    out.clear();
    auto p = splitFrame(frame);
    if (p.empty() || p[0] != "5") return false;
    if (p.size() < 5 || (p.size() - 1) % 4 != 0) return false;
    for (size_t i = 1; i + 3 < p.size(); i += 4) {
        VisionItem it;
        if (!parseInt(p[i], it.id) || !parseInt(p[i + 1], it.x_cm) || !parseInt(p[i + 2], it.y_cm) ||
            !parseInt(p[i + 3], it.yaw_deg)) {
            out.clear();
            return false;
        }
        out.push_back(it);
    }
    return true;
}

// Décode une trame cmd 3. false si invalide (autre commande, nombre impair d'arguments, non numérique).
inline bool parseDepotMessage(const std::string& frame, std::vector<game::DepotAssignment>& out) {
    out.clear();
    auto p = splitFrame(frame);
    if (p.empty() || p[0] != "3") return false;
    if ((p.size() - 1) % 2 != 0) return false;
    for (size_t i = 1; i + 1 < p.size(); i += 2) {
        game::DepotAssignment a{};
        if (!parseInt(p[i], a.pami_id) || !parseInt(p[i + 1], a.zone_id)) {
            out.clear();
            return false;
        }
        out.push_back(a);
    }
    return true;
}

}  // namespace net
