#include "net/WifiLink.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdlib>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace net {

namespace {
constexpr size_t kMaxPeers = 32;  // protège contre un flot d'expéditeurs inconnus
}

std::string Peer::ip() const {
    char buf[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &addr.sin_addr, buf, sizeof(buf));
    return buf;
}

std::string Peer::str() const { return ip() + ":" + std::to_string(ntohs(addr.sin_port)); }

WifiLink::WifiLink(int listenPort, int maxTxErrors)
    : port_(listenPort), maxTxErrors_(maxTxErrors) {}

WifiLink::~WifiLink() { stop(); }

bool WifiLink::open() {
    if (fd_ >= 0) return true;
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::fprintf(stderr, "[net] socket : %s\n", std::strerror(errno));
        return false;
    }
    int yes = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    ::setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        std::fprintf(stderr, "[net] bind port %d : %s\n", port_, std::strerror(errno));
        ::close(fd);
        return false;
    }
    fd_ = fd;
    std::fprintf(stderr, "[net] écoute UDP sur le port %d\n", port_);
    return true;
}

void WifiLink::start() {
    if (fd_ < 0 || running_.exchange(true)) return;
    recvThread_ = std::thread(&WifiLink::recvLoop, this);
}

void WifiLink::stop() {
    running_ = false;
    if (recvThread_.joinable()) recvThread_.join();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool WifiLink::addPeer(const std::string& ipPort) {
    auto colon = ipPort.rfind(':');
    if (colon == std::string::npos) return false;
    Peer p;
    p.addr.sin_family = AF_INET;
    int port = 0;
    try { port = std::stoi(ipPort.substr(colon + 1)); } catch (...) { return false; }
    if (port <= 0 || port > 65535) return false;
    p.addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, ipPort.substr(0, colon).c_str(), &p.addr.sin_addr) != 1) return false;
    learnPeer(p);
    return true;
}

void WifiLink::learnPeer(const Peer& p) {
    if (!p.valid()) return;
    {
        std::lock_guard<std::mutex> lk(peersMtx_);
        for (const auto& q : peers_) if (q.sameAs(p)) return;
        if (peers_.size() >= kMaxPeers) return;
        peers_.push_back(p);
    }
    std::fprintf(stderr, "[net] pair ajouté : %s\n", p.str().c_str());
}

size_t WifiLink::peerCount() const {
    std::lock_guard<std::mutex> lk(peersMtx_);
    return peers_.size();
}

bool WifiLink::sendOne(const Peer& peer, const std::string& data) {
    if (fd_ < 0 || !peer.valid()) return false;
    ssize_t n = ::sendto(fd_, data.data(), data.size(), MSG_DONTWAIT,
                         reinterpret_cast<const sockaddr*>(&peer.addr), sizeof(peer.addr));
    if (n == static_cast<ssize_t>(data.size())) {
        txErrors_ = 0;
        healthy_ = true;
        return true;
    }
    if (++txErrors_ >= maxTxErrors_ && healthy_.exchange(false))
        std::fprintf(stderr, "[net] %d erreurs d'envoi consécutives -> unhealthy (%s)\n",
                     txErrors_.load(), std::strerror(errno));
    return false;
}

void WifiLink::send(const std::string& frame) {
    if (frame.empty()) return;
    const std::string data = frame + "\n";
    std::vector<Peer> copy;
    {
        std::lock_guard<std::mutex> lk(peersMtx_);
        copy = peers_;
    }
    for (const auto& p : copy) sendOne(p, data);
}

void WifiLink::sendTo(const Peer& peer, const std::string& frame) {
    if (frame.empty()) return;
    sendOne(peer, frame + "\n");
}

void WifiLink::reply(const Peer& peer, const std::string& frame) {
    if (peer.valid()) sendTo(peer, frame);
    else send(frame);
}

void WifiLink::registerHandler(int cmd, Handler h) {
    std::lock_guard<std::mutex> lk(hMtx_);
    for (auto& kv : handlers_) {
        if (kv.first == cmd) { kv.second = std::move(h); return; }
    }
    handlers_.emplace_back(cmd, std::move(h));
}

void WifiLink::injectRx(const std::string& line) { dispatchLine(line, Peer{}); }

void WifiLink::recvLoop() {
    char buf[1500];
    while (running_) {
        pollfd p{fd_, POLLIN, 0};
        if (::poll(&p, 1, 100) <= 0) continue;

        sockaddr_in from{};
        socklen_t len = sizeof(from);
        ssize_t n = ::recvfrom(fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &len);
        if (n <= 0) continue;

        Peer peer;
        peer.addr = from;
        learnPeer(peer);

        // Un datagramme peut contenir plusieurs lignes ; chaque datagramme est indépendant.
        std::string data(buf, static_cast<size_t>(n));
        std::stringstream ss(data);
        std::string line;
        while (std::getline(ss, line)) {
            while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
            if (!line.empty()) dispatchLine(line, peer);
        }
    }
}

void WifiLink::dispatchLine(const std::string& line, const Peer& from) {
    std::vector<std::string> parts;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, '|')) parts.push_back(tok);
    if (parts.empty()) return;
    char* end = nullptr;
    const long cmdValue = std::strtol(parts[0].c_str(), &end, 10);
    if (parts[0].empty() || *end != '\0') return;
    const int cmd = static_cast<int>(cmdValue);
    parts.erase(parts.begin());
    Handler h;
    {
        std::lock_guard<std::mutex> lk(hMtx_);
        for (const auto& kv : handlers_) {
            if (kv.first == cmd) { h = kv.second; break; }
        }
    }
    if (h) h(parts, from);
}

}  // namespace net
