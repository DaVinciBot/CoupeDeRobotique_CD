// Démo A* : à lancer et à modifier pour comprendre l'algorithme. Aucune dépendance (pas d'OpenCV).
//
// Compilation :  g++ -std=c++17 -O2 -Isrc examples/astar_demo.cpp -o astar_demo
//
// Usage :  ./astar_demo [options]
//   --steps N      affiche les N premières cases développées avec leurs g, h, f
//   --dijkstra     affiche la carte de Dijkstra au lieu de celle d'A*
//   --weight W     poids de l'heuristique (1 = A*, 0 = Dijkstra, 5 = glouton)
//   --no-diag      déplacements en 4 directions seulement
//   --inflate R    grossit les obstacles de R cases (marge de sécurité du robot)
//   --map FICHIER  charge une carte texte ('#' obstacle, 'S' départ, 'G' arrivée, '.' libre)
//
// Légende de l'affichage :  S départ   G arrivée   # obstacle   x marge de sécurité
//                           * chemin trouvé   o case explorée   . case libre non visitée
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "planning/AStar.hpp"

using planning::Cell;
using planning::Grid;
using planning::Options;
using planning::Result;

// Carte par défaut : 30 x 14 cases. Imaginez 10 cm par case : une table de 3 m x 1,4 m.
// Un grand mur sépare le départ de l'arrivée ; le seul passage est en bas.
static const std::vector<std::string> kDefaultMap = {
    "..............#...............",
    "..............#...............",
    ".S............#...............",
    "..............#..........G....",
    "....#####.....#...............",
    "....#...#.....#.....####......",
    "....#...#.....#.....#..#......",
    "..............#.....#..#......",
    "..............#...............",
    "..............#...............",
    "..............................",
    "..............................",
    "..............................",
    "..............................",
};

struct Scene {
    std::vector<std::string> lines;
    Grid grid{0, 0};
    Cell start{-1, -1}, goal{-1, -1};
};

static bool buildScene(const std::vector<std::string>& lines, Scene& s) {
    size_t w = 0;
    for (const auto& l : lines) w = std::max(w, l.size());
    if (w == 0 || lines.empty()) return false;
    s.lines = lines;
    for (auto& l : s.lines) l.resize(w, '.');  // complète les lignes trop courtes
    s.grid = Grid(static_cast<int>(w), static_cast<int>(lines.size()));
    for (int y = 0; y < s.grid.height(); ++y)
        for (int x = 0; x < s.grid.width(); ++x) {
            char c = s.lines[y][x];
            if (c == '#') s.grid.setBlocked(x, y);
            else if (c == 'S') s.start = {x, y};
            else if (c == 'G') s.goal = {x, y};
        }
    return s.start.x >= 0 && s.goal.x >= 0;
}

static void draw(const Scene& s, const Grid& used, const Result& r) {
    std::vector<std::string> canvas = s.lines;
    for (auto& row : canvas)
        for (auto& c : row)
            if (c != '#') c = '.';
    for (int y = 0; y < used.height(); ++y)
        for (int x = 0; x < used.width(); ++x)
            if (!used.isFree(x, y) && canvas[y][x] != '#') canvas[y][x] = 'x';
    for (const auto& st : r.expanded) canvas[st.cell.y][st.cell.x] = 'o';
    for (const auto& c : r.path) canvas[c.y][c.x] = '*';
    canvas[s.start.y][s.start.x] = 'S';
    canvas[s.goal.y][s.goal.x] = 'G';
    for (const auto& row : canvas) std::printf("  %s\n", row.c_str());
}

static void summary(const char* name, const Result& r) {
    if (!r.found) {
        std::printf("  %-22s aucun chemin\n", name);
        return;
    }
    std::printf("  %-22s coût %.2f | %3zu cases dans le chemin | %4zu cases explorées\n", name, r.cost,
                r.path.size(), r.expanded.size());
}

int main(int argc, char** argv) {
    Options opt;
    int steps = 0, inflate = 0;
    bool showDijkstra = false;
    std::string mapFile;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* what) -> const char* {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s attend une valeur\n", what); std::exit(1); }
            return argv[++i];
        };
        if (a == "--steps") steps = std::atoi(next("--steps"));
        else if (a == "--weight") opt.hWeight = std::atof(next("--weight"));
        else if (a == "--inflate") inflate = std::atoi(next("--inflate"));
        else if (a == "--map") mapFile = next("--map");
        else if (a == "--no-diag") opt.diagonal = false;
        else if (a == "--dijkstra") showDijkstra = true;
        else { std::fprintf(stderr, "option inconnue : %s (voir l'en-tête du fichier)\n", a.c_str()); return 1; }
    }

    std::vector<std::string> lines = kDefaultMap;
    if (!mapFile.empty()) {
        std::ifstream f(mapFile);
        if (!f) { std::fprintf(stderr, "impossible de lire %s\n", mapFile.c_str()); return 1; }
        lines.clear();
        for (std::string l; std::getline(f, l);) {
            while (!l.empty() && (l.back() == '\r')) l.pop_back();
            if (!l.empty()) lines.push_back(l);
        }
    }
    Scene scene;
    if (!buildScene(lines, scene)) {
        std::fprintf(stderr, "la carte doit contenir un 'S' (départ) et un 'G' (arrivée)\n");
        return 1;
    }

    Grid used = inflate > 0 ? scene.grid.inflated(inflate) : scene.grid;
    used.setBlocked(scene.start.x, scene.start.y, false);  // départ et arrivée toujours libres
    used.setBlocked(scene.goal.x, scene.goal.y, false);

    Options astarOpt = opt;
    Options dijkstraOpt = opt;
    dijkstraOpt.hWeight = 0.0;
    Result astar = planning::findPath(used, scene.start, scene.goal, astarOpt);
    Result dijkstra = planning::findPath(used, scene.start, scene.goal, dijkstraOpt);

    const Result& shown = showDijkstra ? dijkstra : astar;
    std::printf("\n%s  (grille %dx%d, %s, poids heuristique %.1f)\n\n",
                showDijkstra ? "Carte de Dijkstra" : "Carte d'A*", used.width(), used.height(),
                opt.diagonal ? "8 directions" : "4 directions", showDijkstra ? 0.0 : opt.hWeight);
    draw(scene, used, shown);

    std::printf("\nComparaison :\n");
    summary(opt.hWeight == 1.0 ? "A* (h x 1)" : "Heuristique pondérée", astar);
    summary("Dijkstra (h = 0)", dijkstra);
    if (astar.found && dijkstra.found)
        std::printf("\n  A* explore %.0f %% de cases en moins que Dijkstra pour un chemin %s.\n",
                    100.0 * (1.0 - double(astar.expanded.size()) / double(dijkstra.expanded.size())),
                    astar.cost <= dijkstra.cost + 1e-9 ? "de même longueur (optimal)" : "plus long (non optimal)");

    if (steps > 0) {
        std::printf("\nPremières cases développées (%s) :\n", showDijkstra ? "Dijkstra" : "A*");
        const auto& ex = shown.expanded;
        for (int i = 0; i < steps && i < static_cast<int>(ex.size()); ++i)
            std::printf("  #%-3d (%2d,%2d)  g=%6.2f  h=%6.2f  f=%6.2f\n", i + 1, ex[i].cell.x, ex[i].cell.y,
                        ex[i].g, ex[i].h, ex[i].f);
    }
    std::printf("\n");
    return astar.found ? 0 : 2;
}
