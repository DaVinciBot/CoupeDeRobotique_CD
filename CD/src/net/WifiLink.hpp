#pragma once
// Transport réseau UDP : remplace le module LoRa.
// Fonctionne sur n'importe quel lien IP (Wi-Fi 6E, Wi-Fi 5, Ethernet...) : la technologie radio
// se règle au niveau du système (NetworkManager), pas dans ce code.
//
// Les trames restent du texte "cmd|arg|arg\n", un datagramme UDP = une ou plusieurs lignes.
//
// Pairs (robot, PAMIs...) :
//   - statiques : liste ip:port donnée dans le .env (WIFI_PEERS), adresse de broadcast acceptée ;
//   - appris : toute machine qui écrit à la Jetson est mémorisée et recevra les envois suivants.
#include <netinet/in.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace net {

struct Peer {
    sockaddr_in addr{};

    bool valid() const { return addr.sin_port != 0; }
    std::string ip() const;   // "192.168.1.20"
    std::string str() const;  // "192.168.1.20:5006"
    bool sameAs(const Peer& o) const {
        return addr.sin_addr.s_addr == o.addr.sin_addr.s_addr && addr.sin_port == o.addr.sin_port;
    }
};

// args : arguments de la trame (sans la commande), from : expéditeur (invalide si injecté).
using Handler = std::function<void(const std::vector<std::string>& args, const Peer& from)>;

class WifiLink {
public:
    WifiLink(int listenPort, int maxTxErrors);
    ~WifiLink();

    bool open();   // crée le socket UDP et l'attache au port d'écoute
    void start();  // lance le thread de réception
    void stop();

    // "ip:port" (ex. "192.168.1.20:5006" ou "192.168.1.255:5006" pour un broadcast)
    bool addPeer(const std::string& ipPort);

    // Envoi (sans '\n' final) : à tous les pairs / à un pair précis.
    void send(const std::string& frame);
    void sendTo(const Peer& peer, const std::string& frame);
    // Répond à `peer` s'il est valide, sinon envoie à tous (cas du clavier / tests).
    void reply(const Peer& peer, const std::string& frame);

    void registerHandler(int cmd, Handler h);
    // Injecte une trame comme si elle arrivait du réseau (tests, clavier).
    void injectRx(const std::string& line);

    // false après N erreurs d'envoi consécutives, true dès qu'un envoi réussit
    bool healthy() const { return healthy_; }
    size_t peerCount() const;

private:
    void recvLoop();
    void dispatchLine(const std::string& line, const Peer& from);
    void learnPeer(const Peer& p);
    bool sendOne(const Peer& peer, const std::string& data);

    int port_;
    int maxTxErrors_;
    int fd_ = -1;

    std::atomic<bool> running_{false};
    std::atomic<bool> healthy_{true};
    std::atomic<int> txErrors_{0};
    std::thread recvThread_;

    mutable std::mutex peersMtx_;
    std::vector<Peer> peers_;

    std::mutex hMtx_;
    std::vector<std::pair<int, Handler>> handlers_;
};

}  // namespace net
