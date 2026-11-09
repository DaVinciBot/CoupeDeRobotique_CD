#pragma once
// Lecteur minimal de fichier .env (KEY=VALUE). Les variables d'environnement
// du process sont prioritaires sur le fichier.
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace utils {

class Config {
public:
    bool load(const std::string& path) {
        std::ifstream f(path);
        if (!f) return false;
        std::string line;
        while (std::getline(f, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') continue;
            auto eq = line.find('=');
            if (eq == std::string::npos) continue;
            kv_[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
        }
        return true;
    }

    std::string str(const std::string& key, const std::string& def = "") const {
        if (const char* e = std::getenv(key.c_str())) return e;
        auto it = kv_.find(key);
        return it == kv_.end() ? def : it->second;
    }
    int integer(const std::string& key, int def) const {
        auto s = str(key);
        if (s.empty()) return def;
        try { return std::stoi(s); } catch (...) { return def; }
    }
    double real(const std::string& key, double def) const {
        auto s = str(key);
        if (s.empty()) return def;
        try { return std::stod(s); } catch (...) { return def; }
    }
    bool boolean(const std::string& key, bool def) const {
        auto s = str(key);
        if (s.empty()) return def;
        return s == "1" || s == "true" || s == "TRUE" || s == "yes";
    }
    std::vector<int> intList(const std::string& key, const std::vector<int>& def = {}) const {
        auto s = str(key);
        if (s.empty()) return def;
        std::vector<int> out;
        std::stringstream ss(s);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            try { out.push_back(std::stoi(trim(tok))); } catch (...) {}
        }
        return out;
    }

private:
    static std::string trim(const std::string& s) {
        const char* ws = " \t\r\n";
        auto b = s.find_first_not_of(ws);
        if (b == std::string::npos) return "";
        auto e = s.find_last_not_of(ws);
        return s.substr(b, e - b + 1);
    }
    std::unordered_map<std::string, std::string> kv_;
};

}  // namespace utils
