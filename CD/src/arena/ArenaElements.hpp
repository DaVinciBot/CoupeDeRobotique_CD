#pragma once
// Géométrie de l'arène (repère : origine coin haut-gauche, x vers la droite, y vers le bas, en mètres).
// ATTENTION : valeurs PLACEHOLDER à vérifier / mettre à jour selon le règlement et la table.
#include <map>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace arena {

struct Zone {
    int id_zone;
    std::string name;
    cv::Point2f origin;   // coin haut-gauche (m)
    cv::Size2f size;      // (largeur, hauteur) (m)
    cv::Scalar color;     // BGR
    char team;            // 'B', 'Y' ou '*' (les deux)

    bool contains(const cv::Point2f& p) const {
        return p.x >= origin.x && p.x <= origin.x + size.width &&
               p.y >= origin.y && p.y <= origin.y + size.height;
    }
};

struct ArenaElements {
    float width = 3.0f;
    float height = 2.0f;
    std::vector<Zone> starting_zone;
    std::vector<Zone> zone_depot;
    std::vector<Zone> zone_ramassage;
    // ID ArUco de référence -> position réelle (m)
    std::map<int, cv::Point2f> refMarkers;

    static const Zone* findZone(const std::vector<Zone>& zones, const cv::Point2f& p) {
        for (const auto& z : zones) if (z.contains(p)) return &z;
        return nullptr;
    }
};

inline ArenaElements makeArena() {
    ArenaElements a;
    a.starting_zone = {
        {1, "start_B", {0.00f, 0.00f}, {0.45f, 0.45f}, {255, 120, 0}, 'B'},
        {2, "start_Y", {2.55f, 0.00f}, {0.45f, 0.45f}, {0, 200, 255}, 'Y'},
    };
    a.zone_depot = {
        {10, "depot_B1", {0.00f, 1.55f}, {0.45f, 0.45f}, {255, 160, 60}, 'B'},
        {11, "depot_B2", {0.50f, 1.55f}, {0.45f, 0.45f}, {255, 160, 60}, 'B'},
        {12, "depot_Y1", {2.55f, 1.55f}, {0.45f, 0.45f}, {60, 220, 255}, 'Y'},
        {13, "depot_Y2", {2.05f, 1.55f}, {0.45f, 0.45f}, {60, 220, 255}, 'Y'},
    };
    a.zone_ramassage = {
        {20, "pickup_1", {1.10f, 0.60f}, {0.35f, 0.35f}, {120, 200, 120}, '*'},
        {21, "pickup_2", {1.55f, 0.60f}, {0.35f, 0.35f}, {120, 200, 120}, '*'},
    };
    // Marqueurs de référence (placeholders : ids 20..23 aux 4 coins utiles de la table)
    a.refMarkers = {
        {20, {0.60f, 0.60f}},
        {21, {2.40f, 0.60f}},
        {22, {0.60f, 1.40f}},
        {23, {2.40f, 1.40f}},
    };
    return a;
}

}  // namespace arena
