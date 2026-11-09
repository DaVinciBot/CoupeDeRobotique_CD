// Tests des formats de messages (cmd 3 et 5), de MatchState et de la couche réseau WifiLink.
// Si vous changez le protocole pour l'aligner sur votre robot, mettez à jour les valeurs attendues ici :
// ces tests documentent le format exact envoyé.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "arena/ArenaElements.hpp"
#include "check.hpp"
#include "game/MatchState.hpp"
#include "net/Protocol.hpp"
#include "net/WifiLink.hpp"

namespace {

constexpr float kPi = 3.14159265358979f;

detector::Detection det(int id, float x, float y, float yawRad) {
    detector::Detection d;
    d.id = id;
    d.pos = {x, y};
    d.yaw = yawRad;
    return d;
}

// ---------------------------------------------------------------- cmd 5 (vision) ----------------

void testVisionMessageFormat() {
    CHECK_EQ(net::buildVisionMessage({}), std::string(""));  // rien à envoyer
    CHECK_EQ(net::buildVisionMessage({det(36, 1.234f, 0.5f, kPi / 2)}), std::string("5|36|123|50|90"));
    CHECK_EQ(net::buildVisionMessage({det(36, 1.234f, 0.5f, kPi / 2), det(37, 0.f, 2.f, -kPi / 4)}),
             std::string("5|36|123|50|90|37|0|200|-45"));
    CHECK_EQ(net::buildVisionMessage({det(5, -0.02f, 1.f, 3.0f)}), std::string("5|5|-2|100|172"));
}

void testVisionMessageLimits() {
    std::vector<detector::Detection> many;
    for (int i = 0; i < 100; ++i) many.push_back(det(i, 2.99f, 1.99f, -3.1f));
    std::string msg = net::buildVisionMessage(many);
    std::vector<net::VisionItem> items;
    CHECK(net::parseVisionMessage(msg, items));
    CHECK_EQ(items.size(), net::kMaxVisionItems);         // tronqué à 40 marqueurs
    CHECK(msg.size() < 1400);                              // tient dans un datagramme UDP (MTU ~1500)
    CHECK(msg.find('\n') == std::string::npos);            // le saut de ligne est ajouté par WifiLink
    CHECK(msg.find(' ') == std::string::npos);

    CHECK_EQ(net::parseVisionMessage(net::buildVisionMessage(many, 3), items) ? items.size() : 0u, 3u);
}

void testVisionRoundTrip() {
    std::vector<detector::Detection> in = {det(36, 1.234f, 0.5f, 0.3f), det(40, 2.9f, 1.9f, -3.0f),
                                           det(12, 0.01f, 0.02f, 0.f)};
    std::vector<net::VisionItem> out;
    CHECK(net::parseVisionMessage(net::buildVisionMessage(in), out));
    CHECK_EQ(out.size(), in.size());
    for (size_t i = 0; i < in.size() && i < out.size(); ++i) {
        CHECK_EQ(out[i].id, in[i].id);
        CHECK_NEAR(out[i].x_cm, in[i].pos.x * 100.0, 0.51);  // arrondi au cm
        CHECK_NEAR(out[i].y_cm, in[i].pos.y * 100.0, 0.51);
        CHECK_NEAR(out[i].yaw_deg, in[i].yaw * 180.0 / kPi, 0.51);  // arrondi au degré
    }
}

void testVisionParserRejectsGarbage() {
    std::vector<net::VisionItem> out;
    CHECK(!net::parseVisionMessage("", out));
    CHECK(!net::parseVisionMessage("5", out));                // aucun marqueur
    CHECK(!net::parseVisionMessage("5|36|12|3", out));        // incomplet
    CHECK(!net::parseVisionMessage("5|36|12|3|90|7", out));   // reste parasite
    CHECK(!net::parseVisionMessage("6|36|12|3|90", out));     // mauvaise commande
    CHECK(!net::parseVisionMessage("5|a|b|c|d", out));        // non numérique
    CHECK(!net::parseVisionMessage("5|36|12|3|9x", out));     // fin parasite
    CHECK(out.empty());
    CHECK(net::parseVisionMessage("5|36|12|3|90", out));
    CHECK_EQ(out.size(), 1u);
}

// ---------------------------------------------------------------- cmd 3 (dépôt) ----------------

void testDepotMessage() {
    using game::DepotAssignment;
    using game::MatchState;
    CHECK_EQ(MatchState::buildMsg3({}), std::string("3"));
    CHECK_EQ(MatchState::buildMsg3({{1, 10}, {2, 11}}), std::string("3|1|10|2|11"));

    std::vector<DepotAssignment> out;
    CHECK(net::parseDepotMessage("3|1|10|2|11", out));
    CHECK_EQ(out.size(), 2u);
    if (out.size() == 2) {
        CHECK_EQ(out[1].pami_id, 2);
        CHECK_EQ(out[1].zone_id, 11);
    }
    CHECK(net::parseDepotMessage("3", out) && out.empty());
    CHECK(!net::parseDepotMessage("3|1|10|2", out));  // nombre impair d'arguments
    CHECK(!net::parseDepotMessage("5|1|10", out));
    CHECK(!net::parseDepotMessage("3|a|b", out));
}

// ---------------------------------------------------------------- MatchState -------------------

void testPamiRegistration() {
    game::MatchState m;
    int a = m.registerPami("PAMI-A");
    int b = m.registerPami("PAMI-B");
    CHECK(a != b);
    CHECK_EQ(m.registerPami("PAMI-A"), a);  // même clé -> même ID (robot qui redémarre)
    CHECK_EQ(m.registerPami("PAMI-B"), b);
    int c1 = m.registerPami(""), c2 = m.registerPami("");
    CHECK(c1 != c2);  // sans clé : un nouvel ID à chaque fois
    CHECK_EQ(m.pamiIds().size(), 4u);
}

void testMatchLifecycle() {
    game::MatchState m;
    CHECK(!m.started());
    CHECK(!m.isOver());
    CHECK(!m.shouldSendPreEnd());
    CHECK_NEAR(m.elapsed(), 0.0, 1e-9);

    m.startMatch('Y');
    CHECK(m.started());
    CHECK_EQ(m.color(), 'Y');
    CHECK(!m.isOver());
    CHECK(!m.shouldSendPreEnd());  // vient de démarrer
    CHECK(m.elapsed() < 1.0);

    m.startMatch('x');  // couleur inconnue -> B par défaut
    CHECK_EQ(m.color(), 'B');
    // NB : l'écoulement du temps (fin à 100 s, pré-fin à 90 s) n'est pas testé ici car il n'y a pas
    // d'horloge injectable ; à ajouter si vous rendez MatchState configurable.
}

void testDepotAssignments() {
    const auto arena = arena::makeArena();
    for (char color : {'B', 'Y'}) {
        game::MatchState m;
        std::vector<int> ids;
        for (const char* key : {"A", "B", "C"}) ids.push_back(m.registerPami(key));
        m.startMatch(color);
        auto assign = m.computeDepotAssignments(arena);

        CHECK_EQ(assign.size(), ids.size());
        std::vector<int> allowed;
        for (const auto& z : arena.zone_depot)
            if (z.team == color || z.team == '*') allowed.push_back(z.id_zone);
        CHECK(!allowed.empty());
        for (size_t i = 0; i < assign.size(); ++i) {
            CHECK_EQ(assign[i].pami_id, ids[i]);
            bool ok = false;
            for (int z : allowed) ok = ok || z == assign[i].zone_id;
            CHECK(ok);  // un PAMI jaune ne reçoit jamais une zone bleue
        }
        if (allowed.size() == 2 && assign.size() == 3) CHECK_EQ(assign[2].zone_id, assign[0].zone_id);  // tour de rôle
    }
}

// ---------------------------------------------------------------- WifiLink (sans réseau) -------

void testDispatch() {
    net::WifiLink link(0, 5);  // pas de open() : on injecte directement les trames
    std::vector<std::string> lastArgs;
    int calls1 = 0, calls4 = 0, callsOld = 0;
    bool fromValid = true;

    link.registerHandler(1, [&](const std::vector<std::string>&, const net::Peer&) { ++callsOld; });
    link.registerHandler(1, [&](const std::vector<std::string>& a, const net::Peer& from) {  // remplace le précédent
        ++calls1;
        lastArgs = a;
        fromValid = from.valid();
    });
    link.registerHandler(4, [&](const std::vector<std::string>& a, const net::Peer&) {
        ++calls4;
        lastArgs = a;
    });

    link.injectRx("1|PAMI-A");
    CHECK_EQ(calls1, 1);
    CHECK_EQ(lastArgs.size(), 1u);
    if (!lastArgs.empty()) CHECK_EQ(lastArgs[0], std::string("PAMI-A"));
    CHECK(!fromValid);  // injecté : pas d'expéditeur réseau
    CHECK_EQ(callsOld, 0);

    link.injectRx("1");
    CHECK_EQ(calls1, 2);
    CHECK(lastArgs.empty());

    link.injectRx("4|Y");
    CHECK_EQ(calls4, 1);
    if (!lastArgs.empty()) CHECK_EQ(lastArgs[0], std::string("Y"));

    // Entrées invalides : ignorées sans planter ni appeler un handler
    for (const char* junk : {"", "abc", "|", "||", "9|x", "-", "1x|y"}) link.injectRx(junk);
    CHECK_EQ(calls1, 2);
    CHECK_EQ(calls4, 1);
}

void testPeersAndSendWithoutSocket() {
    net::WifiLink link(0, 5);
    CHECK(link.addPeer("192.168.1.20:5006"));
    CHECK(link.addPeer("192.168.1.255:5006"));  // broadcast accepté
    CHECK(!link.addPeer("192.168.1.20"));        // pas de port
    CHECK(!link.addPeer("bad:5006"));
    CHECK(!link.addPeer("1.2.3.4:0"));
    CHECK(!link.addPeer("1.2.3.4:70000"));
    CHECK(!link.addPeer("999.1.1.1:5000"));
    CHECK_EQ(link.peerCount(), 2u);
    CHECK(link.addPeer("192.168.1.20:5006"));    // doublon : accepté mais pas ajouté 2 fois
    CHECK_EQ(link.peerCount(), 2u);
    link.send("5|1|2|3|4");                       // socket fermé : ne doit pas planter
    link.reply(net::Peer{}, "2|1");
    CHECK(link.healthy());
}

// ---------------------------------------------------------------- WifiLink (vrai UDP local) -----

void testRealUdpLoopback() {
    constexpr int kPort = 47123;
    net::WifiLink link(kPort, 5);
    if (!link.open()) {
        std::printf("  SKIP : port %d indisponible\n", kPort);
        return;
    }
    game::MatchState match;
    link.registerHandler(1, [&](const std::vector<std::string>& args, const net::Peer& from) {
        std::string key = !args.empty() ? args[0] : from.ip();
        link.reply(from, "2|" + std::to_string(match.registerPami(key)));
    });
    link.start();

    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    timeval tv{1, 0};
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(kPort);
    inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);

    auto ask = [&](const std::string& payload) {
        ::sendto(sock, payload.data(), payload.size(), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    };
    auto receive = [&]() {
        char buf[256];
        ssize_t n = ::recv(sock, buf, sizeof(buf), 0);
        return n > 0 ? std::string(buf, static_cast<size_t>(n)) : std::string("<timeout>");
    };

    ask("1|KEY-A\n");
    CHECK_EQ(receive(), std::string("2|1\n"));  // 1re demande : ID 1, réponse terminée par '\n'
    ask("1|KEY-A\r\n");                          // retour chariot Windows toléré, même clé -> même ID
    CHECK_EQ(receive(), std::string("2|1\n"));
    ask("1|KEY-B\n1|KEY-C\n");                   // 2 trames dans un datagramme -> 2 réponses
    CHECK_EQ(receive(), std::string("2|2\n"));
    CHECK_EQ(receive(), std::string("2|3\n"));

    // L'expéditeur est mémorisé : un envoi à tous les pairs lui parvient.
    CHECK_EQ(link.peerCount(), 1u);
    link.send("5|36|150|100|0");
    CHECK_EQ(receive(), std::string("5|36|150|100|0\n"));

    ::close(sock);
    link.stop();
}

}  // namespace

int main() {
    RUN(testVisionMessageFormat);
    RUN(testVisionMessageLimits);
    RUN(testVisionRoundTrip);
    RUN(testVisionParserRejectsGarbage);
    RUN(testDepotMessage);
    RUN(testPamiRegistration);
    RUN(testMatchLifecycle);
    RUN(testDepotAssignments);
    RUN(testDispatch);
    RUN(testPeersAndSendWithoutSocket);
    RUN(testRealUdpLoopback);
    return testing_util::finish("test_protocol");
}
