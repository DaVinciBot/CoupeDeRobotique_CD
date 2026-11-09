#pragma once
// Recherche de chemin A* sur une grille d'occupation (header-only, sans dépendance).
//
//   f = g + h
//   g : coût réel parcouru depuis le départ
//   h : estimation du coût restant jusqu'à l'arrivée (heuristique)
//   A* développe toujours la case ouverte qui a le plus petit f.
//
// Avec hWeight = 0 l'heuristique disparaît : c'est l'algorithme de Dijkstra (utile pour comparer).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace planning {

struct Cell {
    int x = 0;
    int y = 0;
    bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
};

class Grid {
public:
    Grid(int width, int height) : w_(width), h_(height), blocked_(static_cast<size_t>(width) * height, 0) {}

    int width() const { return w_; }
    int height() const { return h_; }
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }
    bool isFree(int x, int y) const { return inside(x, y) && !blocked_[idx(x, y)]; }

    void setBlocked(int x, int y, bool blocked = true) {
        if (inside(x, y)) blocked_[idx(x, y)] = blocked ? 1 : 0;
    }
    // Rectangle inclusif de cases bloquées.
    void blockRect(int x0, int y0, int x1, int y1) {
        for (int y = y0; y <= y1; ++y)
            for (int x = x0; x <= x1; ++x) setBlocked(x, y);
    }
    // Grossit chaque obstacle de r cases : marge de sécurité égale au rayon du robot.
    Grid inflated(int r) const {
        Grid out = *this;
        for (int y = 0; y < h_; ++y)
            for (int x = 0; x < w_; ++x)
                if (!isFree(x, y))
                    for (int dy = -r; dy <= r; ++dy)
                        for (int dx = -r; dx <= r; ++dx) out.setBlocked(x + dx, y + dy);
        return out;
    }

private:
    size_t idx(int x, int y) const { return static_cast<size_t>(y) * w_ + x; }
    int w_, h_;
    std::vector<uint8_t> blocked_;
};

struct Options {
    bool diagonal = true;   // 8 directions (true) ou 4 directions (false)
    double hWeight = 1.0;   // 1 = A* ; 0 = Dijkstra ; >1 = plus glouton (plus rapide, moins optimal)
};

// Une case "développée" (sortie de la liste ouverte), dans l'ordre d'exploration.
struct Step {
    Cell cell;
    double g = 0, h = 0, f = 0;
};

struct Result {
    bool found = false;
    std::vector<Cell> path;       // de départ à arrivée, inclus
    double cost = 0;              // longueur du chemin en cases (diagonale = racine de 2)
    std::vector<Step> expanded;   // toutes les cases développées, dans l'ordre
};

// Heuristique : distance minimale théorique sans obstacle.
//   4 directions : Manhattan ; 8 directions : octile.
inline double heuristic(const Cell& a, const Cell& b, bool diagonal) {
    const double dx = std::abs(a.x - b.x), dy = std::abs(a.y - b.y);
    if (!diagonal) return dx + dy;
    return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
}

inline Result findPath(const Grid& grid, const Cell& start, const Cell& goal, const Options& opt = {}) {
    Result res;
    if (!grid.isFree(start.x, start.y) || !grid.isFree(goal.x, goal.y)) return res;

    const int W = grid.width(), H = grid.height();
    const size_t N = static_cast<size_t>(W) * H;
    const double INF = std::numeric_limits<double>::infinity();
    auto index = [W](int x, int y) { return static_cast<size_t>(y) * W + x; };

    std::vector<double> g(N, INF);        // meilleur coût connu pour atteindre chaque case
    std::vector<long> parent(N, -1);      // case précédente sur le meilleur chemin
    std::vector<uint8_t> closed(N, 0);    // cases déjà développées

    struct Node {
        double f, h;
        size_t i;
    };
    // File de priorité : plus petit f d'abord ; à f égal, plus petit h (plus proche du but).
    auto worse = [](const Node& a, const Node& b) { return a.f != b.f ? a.f > b.f : a.h > b.h; };
    std::priority_queue<Node, std::vector<Node>, decltype(worse)> open(worse);

    struct Move { int dx, dy; double cost; };
    static const Move kMoves[8] = {{1, 0, 1.0},  {-1, 0, 1.0}, {0, 1, 1.0},  {0, -1, 1.0},
                                   {1, 1, M_SQRT2}, {1, -1, M_SQRT2}, {-1, 1, M_SQRT2}, {-1, -1, M_SQRT2}};
    const int nMoves = opt.diagonal ? 8 : 4;

    const size_t si = index(start.x, start.y), gi = index(goal.x, goal.y);
    g[si] = 0;
    const double h0 = opt.hWeight * heuristic(start, goal, opt.diagonal);
    open.push({h0, h0, si});

    while (!open.empty()) {
        const Node cur = open.top();
        open.pop();
        if (closed[cur.i]) continue;  // entrée périmée (une meilleure a déjà été traitée)
        closed[cur.i] = 1;

        const Cell c{static_cast<int>(cur.i % W), static_cast<int>(cur.i / W)};
        res.expanded.push_back({c, g[cur.i], cur.h, g[cur.i] + cur.h});

        if (cur.i == gi) {
            for (long p = static_cast<long>(gi); p != -1; p = parent[p])
                res.path.push_back({static_cast<int>(p % W), static_cast<int>(p / W)});
            std::reverse(res.path.begin(), res.path.end());
            res.cost = g[gi];
            res.found = true;
            return res;
        }

        for (int m = 0; m < nMoves; ++m) {
            const int nx = c.x + kMoves[m].dx, ny = c.y + kMoves[m].dy;
            if (!grid.isFree(nx, ny)) continue;
            // Diagonale : on ne "coupe" pas le coin d'un obstacle.
            if (kMoves[m].dx != 0 && kMoves[m].dy != 0 &&
                (!grid.isFree(c.x + kMoves[m].dx, c.y) || !grid.isFree(c.x, c.y + kMoves[m].dy)))
                continue;
            const size_t ni = index(nx, ny);
            if (closed[ni]) continue;
            const double ng = g[cur.i] + kMoves[m].cost;
            if (ng < g[ni]) {
                g[ni] = ng;
                parent[ni] = static_cast<long>(cur.i);
                const double h = opt.hWeight * heuristic({nx, ny}, goal, opt.diagonal);
                open.push({ng + h, h, ni});
            }
        }
    }
    return res;  // aucun chemin
}

}  // namespace planning
