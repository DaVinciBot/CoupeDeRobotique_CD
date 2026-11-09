#include "game/MatchState.hpp"

#include <algorithm>

namespace game {

void MatchState::startMatch(char color) {
    std::lock_guard<std::mutex> lk(mtx_);
    color_ = (color == 'Y' || color == 'y') ? 'Y' : 'B';
    t0_ = std::chrono::steady_clock::now();
    started_ = true;
    preEndSent_ = false;
}

bool MatchState::started() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return started_;
}

char MatchState::color() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return color_;
}

double MatchState::elapsed() const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!started_) return 0.0;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
}

bool MatchState::isOver() const { return started() && elapsed() >= MATCH_DURATION; }

bool MatchState::shouldSendPreEnd() {
    double e = elapsed();
    std::lock_guard<std::mutex> lk(mtx_);
    if (!started_ || preEndSent_) return false;
    if (e >= MATCH_DURATION - PRE_END_OFFSET) { preEndSent_ = true; return true; }
    return false;
}

int MatchState::registerPami(const std::string& key) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!key.empty()) {
        auto it = keyToId_.find(key);
        if (it != keyToId_.end()) return it->second;
    }
    int id = nextId_++;
    keyToId_[key.empty() ? "#anon" + std::to_string(id) : key] = id;
    return id;
}

std::vector<int> MatchState::pamiIds() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<int> ids;
    for (const auto& kv : keyToId_) ids.push_back(kv.second);
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<DepotAssignment> MatchState::computeDepotAssignments(
    const arena::ArenaElements& arena) const {
    std::vector<arena::Zone> zones;
    char col = color();
    for (const auto& z : arena.zone_depot)
        if (z.team == col || z.team == '*') zones.push_back(z);

    std::vector<DepotAssignment> out;
    auto ids = pamiIds();
    if (zones.empty()) return out;
    for (size_t i = 0; i < ids.size(); ++i)
        out.push_back({ids[i], zones[i % zones.size()].id_zone});
    return out;
}

std::string MatchState::buildMsg3(const std::vector<DepotAssignment>& a) {
    std::string s = "3";
    for (const auto& x : a) s += "|" + std::to_string(x.pami_id) + "|" + std::to_string(x.zone_id);
    return s;
}

}  // namespace game
