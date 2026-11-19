# remote_compute (C++17)

Calcul déporté pour robots : vision ArUco (caméra **ZED en USB**, un seul capteur), projection dans le repère de l'arène,
coordination des PAMIs et communication réseau **UDP sur Wi-Fi 6E** (ou tout autre lien IP).
Portage C++ de la version Python ;

## Structure

```
CMakeLists.txt
.env.example            -> copier en .env
src/
  main.cpp              orchestrateur (boucle de détection)
  camera/ZedCamera      capture ZED en USB (UVC/V4L2) threadée, un seul capteur (gauche par défaut)
  detector/ArucoDetector  détection, homographie/affine, warm-up, carry-forward, rendu 2D
  arena/ArenaElements   géométrie de l'arène (header-only, valeurs placeholder)
  game/MatchState       cycle de match, enregistrement PAMIs, assignations (cmd 3)
  net/WifiLink          transport UDP : pairs, envoi, réception threadée, handlers par commande
  net/Protocol.hpp      format des messages cmd 3 et 5 (construction + décodage)
  planning/AStar.hpp    recherche de chemin A* sur grille (header-only, pas encore branchée au main)
  utils/                Config (.env), Timer (PROFILE_SCOPE), Display (GStreamer)
examples/astar_demo.cpp démo A* autonome (sans OpenCV)
tools/test_client.py    client Python (bibliothèque standard) qui simule un PAMI / le robot
tests/                  tests automatiques (détecteur sur images synthétiques, protocole, réseau)
```

## Prérequis (Jetson / Linux)

- g++ >= 7, CMake >= 3.10 (versions d'Ubuntu 18.04 / JetPack 4.6)
- **OpenCV >= 4.7** avec **GStreamer** et le module `objdetect` (API `cv::aruco::ArucoDetector`).
  Vérifier avec `cv::getBuildInformation()`. CMake s'arrête avec un message explicite si OpenCV est trop ancien.
- Un lien IP entre la Jetson et les robots (Wi-Fi 6E, Wi-Fi 5, Ethernet...).

| Carte | JetPack / Ubuntu | OpenCV fourni | À faire |
|---|---|---|---|
| **Jetson Nano P3450** | 4.6.x / 18.04 | 4.1.1, sans ArUco | `tools/install_opencv_jetson_nano.sh` (OpenCV 4.8 dans `/usr/local`, ~2 à 3 h) |
| **Jetson Orin (Nano, NX...)** | 6.x / 22.04 | 4.8 avec GStreamer | rien |

## Build & lancement

Forme compatible avec le CMake 3.10 de la Jetson Nano (`cmake -S . -B build` et `ctest --test-dir`
n'existent qu'à partir de CMake 3.13 et 3.20) :

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
ctest --output-on-failure              # tests automatiques
cd ..
cp .env.example .env
./build/remote_compute                 # détection (ZED : ZED_DEVICE, ZED_RES=376, ZED_FPS=30 dans .env)
```

Clavier en mode détection : taper une trame brute (`1`, `4|B`, `4|Y`) pour simuler un message entrant.

### Jetson Nano P3450 : première installation

1. Connexion au PC : câble **micro-USB de données**, puis `ssh <utilisateur>@192.168.55.1`. Alimentation
   conseillée : jack 5 V 4 A avec le cavalier **J48**. Un câble Ethernet donne l'accès Internet nécessaire à `apt`.
2. Mode performance : `sudo nvpmodel -m 0 && sudo jetson_clocks`.
3. `bash tools/install_opencv_jetson_nano.sh` (dans `tmux`, car c'est long). Le script crée 4 Go de swap
   et compile OpenCV 4.8 sans CUDA ; `WITH_CUDA=1` ajoute les modules CUDA (~6 à 8 h).
4. Brancher la ZED sur un port **USB 3** (bleu), vérifier son numéro avec `ls /dev/video*` et le reporter
   dans `ZED_DEVICE`. Build comme ci-dessus.

`DISPLAY_MODE=local` affiche le rendu 2D de l'arène sur l'écran branché à la Jetson (pas d'affichage à
distance pour l'instant).

## Réseau Wi-Fi (UDP)

La technologie radio (6E, 5 GHz...) se règle dans le système (NetworkManager, point d'accès), pas dans
ce code : le programme envoie et reçoit des datagrammes UDP.

- La Jetson **écoute** sur `WIFI_LISTEN_PORT` (5005 par défaut).
- Elle **envoie** à tous ses pairs : ceux listés dans `WIFI_PEERS` (`ip:port,ip:port`, une adresse de
  broadcast est possible) et toute machine qui lui a écrit (apprise automatiquement, adresse + port source).
- Prévoir des adresses IP fixes (ou des réservations DHCP) pour la Jetson et les robots, et ouvrir le
  port UDP sur la Jetson si un pare-feu est actif.
- Le lien n'est **pas authentifié ni chiffré** : à utiliser sur un réseau fermé dédié aux robots.

### Test sans robot

Sur la Jetson (ou en local) : `./build/remote_compute`, puis depuis n'importe quelle machine :

```bash
python3 tools/test_client.py --jetson 192.168.1.50 --start Y
```

### Protocole (trames texte `cmd|arg|arg`, une par ligne, UDP)

| cmd | sens | contenu |
|-----|------|---------|
| 1 | PAMI -> Jetson | demande d'ID (arg optionnel : identifiant stable, sinon l'adresse IP sert de clé) |
| 2 | Jetson -> PAMI | `2|id` (réponse à l'expéditeur de la cmd 1) |
| 3 | Jetson -> pairs | `3|pami|zone|pami|zone...` assignations de dépôt |
| 4 | robot -> Jetson | départ de match, `4|B` ou `4|Y` |
| 5 | Jetson -> pairs | `5|id|x_cm|y_cm|yaw_deg|...` positions (jusqu'à 40 marqueurs) |

Les formats de 3 et 5 sont **à aligner sur votre protocole existant** (build_lora_message du Python).
Côté Python : `socket.socket(AF_INET, SOCK_DGRAM)` + `recvfrom` suffit (voir `tools/test_client.py`).

## Démo A*

Aucune dépendance, compile en une commande :

```bash
g++ -std=c++17 -O2 -Isrc examples/astar_demo.cpp -o astar_demo
./astar_demo                 # carte A* + comparaison avec Dijkstra
./astar_demo --steps 10      # détail des 10 premières cases développées (g, h, f)
./astar_demo --dijkstra      # carte de Dijkstra : beaucoup plus de cases explorées
./astar_demo --weight 5      # heuristique trop "gourmande" : plus rapide, chemin moins bon
./astar_demo --no-diag       # 4 directions au lieu de 8
./astar_demo --inflate 1     # grossit les obstacles (marge de sécurité = rayon du robot)
./astar_demo --map ma_carte.txt   # votre carte : '#' obstacle, 'S' départ, 'G' arrivée
```

Pour ne compiler que la démo avec CMake : `cd build && cmake .. -DBUILD_REMOTE_COMPUTE=OFF && make`.
La bibliothèque `src/planning/AStar.hpp` est réutilisable telle quelle : il suffit de remplir une `Grid`
(par exemple 1 case = 5 cm, kaplas et robots détectés = cases bloquées) et d'appeler `planning::findPath`.

## Tests

```bash
cd build && make -j4 && ctest --output-on-failure
```

- `test_detector` : détection ArUco sur images synthétiques (position, orientation, filtre anti-fausses
  détections, report des kaplas, références masquées, caméra en perspective, bruit et ombre).
- `test_protocol` : format des messages cmd 3 et 5, `MatchState`, `WifiLink` (dont un échange UDP local).

Ils ne nécessitent ni caméra ni Jetson, mais **ne remplacent pas un essai sur le matériel réel**.

### Test manuel de la caméra ZED (USB)

`test_zed_camera` (compilé avec le reste, mais pas lancé par `ctest`) affiche le flux d'**un seul**
capteur de la ZED (gauche par défaut). En USB, la ZED est une webcam UVC qui renvoie les deux images
côte à côte ; le SDK ZED n'est pas nécessaire.

```bash
./build/test_zed_camera                  # /dev/video0, 672x376 @ 30 i/s (meilleur réglage mesuré)
./build/test_zed_camera --device 2 --res 1080 --right
./build/test_zed_camera --no-aruco       # sans détection ArUco
```

La détection ArUco (dictionnaire `DICT_4X4_100`) s'active ou se désactive avec `ENABLE_ARUCO_CHECK` en
haut du fichier, les options `--aruco` / `--no-aruco`, ou la touche `a` pendant l'exécution. Chaque
marqueur détecté est signalé dans le terminal (`>>> Marqueur ArUco DÉTECTÉ : id N`) et entouré à l'écran.

L'image est affichée via GStreamer (OpenCV est compilé sans GTK, `cv::imshow` n'est pas disponible) ;
les touches se tapent donc **dans le terminal** : `a` ArUco on/off, `s` capture, `q` ou Ctrl+C pour
quitter. `--no-display` lance le test sans fenêtre.

## État du projet : ce qui est incomplet

Ce code est une base de travail, **pas une version prête pour un match**.

### À remplacer avant tout usage réel

- **Géométrie de l'arène** (`src/arena/ArenaElements.hpp`) : dimensions, zones, IDs et positions des
  marqueurs de référence sont des valeurs d'exemple. Toutes les positions calculées en dépendent.
- **Format des messages** (`src/net/Protocol.hpp`, `MatchState::buildMsg3`) : cmd 3 et cmd 5 sont
  provisoires, à aligner sur le code du robot (puis mettre à jour `tests/test_protocol.cpp`).
- **IDs des kaplas** (`KAPLA_IDS` dans `.env`) : à vérifier.

### Non validé sur le matériel réel

- Programme principal avec la ZED sur la Jetson Nano (seul `test_zed_camera` a été essayé avec la caméra).
- Détection ArUco sur de vraies images : testée uniquement sur des images synthétiques (sans distorsion
  d'objectif, éclairage réel ni flou de mouvement). Les paramètres de détection, volontairement très
  permissifs, sont à régler.
- Performances sur Jetson (cible : 3 à 5 images/s) : à mesurer avec `DEBUG=1`.
- Liaison Wi-Fi 6E : portée, latence et pertes de paquets en conditions de compétition.

### Fonctionnalités non portées ou non terminées

- **CLAHE CUDA** : non implémenté (version CPU uniquement ; TODO dans `ArucoDetector.cpp`).
- **Regroupement des kaplas par zone** (`find_zone_for_kapla` / `group_kapla_by_zone` du Python) : non
  porté ; `ArenaElements::findZone` est disponible pour le faire.
- **Assignation des PAMIs** : simple répartition à tour de rôle des zones de dépôt de l'équipe.
- **A\*** (`src/planning/AStar.hpp`) : fonctionne et est testé via la démo, mais n'est pas branché sur
  les détections (il reste à construire la grille d'occupation à partir des marqueurs).
- **Calibration de la caméra** : retirée pour l'instant. Les images ne sont pas corrigées de la
  distorsion de l'objectif, ce qui fausse un peu les positions sur les bords de l'image.
- **Caméra CSI** : retirée pour l'instant, seule la ZED en USB est gérée.
- **Affichage à distance (UDP)** : retiré pour l'instant ; seul l'affichage local reste disponible.
- **Source d'images alternative** (fichier vidéo) : absente ; seuls les tests et la démo A\* tournent
  sans caméra.
- **Géométrie de l'arène en dur** dans le code, pas dans un fichier de configuration.

### Limites techniques connues

- **Réseau** : UDP sans acquittement, sans authentification ni chiffrement ; à utiliser sur un réseau
  fermé. Un paquet perdu n'est pas renvoyé (les messages périodiques compensent).
- **Threads** : les handlers réseau s'exécutent dans le thread de réception (à garder courts) ; le thread
  clavier de `main.cpp` est détaché (outil de test, à retirer pour un match).
- **Durée du match** : la fin à 100 s et la pré-fin à 90 s ne sont pas testées automatiquement
  (horloge non injectable dans `MatchState`).
- **Intégration continue** : aucune ; OpenCV >= 4.7 doit être compilé pour construire le projet.
