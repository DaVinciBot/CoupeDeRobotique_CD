#pragma once
// Cycle de vie du match + coordination des PAMIs (thread-safe).
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "arena/ArenaElements.hpp"

namespace game {

struct DepotAssignment {
    int pami_id;
    int zone_id;
};

class MatchState {
public:
    static constexpr double MATCH_DURATION = 100.0;  // s
    static constexpr double PRE_END_OFFSET = 10.0;   // s

    // cmd 4 : départ de match, color = 'B' ou 'Y'
    void startMatch(char color);
    bool started() const;
    char color() const;
    double elapsed() const;
    bool isOver() const;
    bool shouldSendPreEnd();  // true une seule fois, PRE_END_OFFSET avant la fin

    // cmd 1 : enregistrement d'un PAMI. `key` (optionnel) identifie le matériel :
    // un même key redonne le même ID (robot qui redémarre pendant la préparation).
    int registerPami(const std::string& key = "");
    std::vector<int> pamiIds() const;

    // Répartition simple des zones de dépôt (de la couleur de l'équipe) aux PAMIs enregistrés.
    std::vector<DepotAssignment> computeDepotAssignments(const arena::ArenaElements& arena) const;

    // cmd 3 : "3|pami|zone|pami|zone|..."
    static std::string buildMsg3(const std::vector<DepotAssignment>& a);

private:
    mutable std::mutex mtx_;
    bool started_ = false;
    bool preEndSent_ = false;
    char color_ = 'B';
    std::chrono::steady_clock::time_point t0_;
    std::map<std::string, int> keyToId_;
    int nextId_ = 1;
};

}  // namespace game
