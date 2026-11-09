# remote_compute (C++17)

Calcul déporté pour robots : vision ArUco (caméra CSI IMX219), projection dans le repère de l'arène,
coordination des PAMIs et communication réseau **UDP sur Wi-Fi 6E** (ou tout autre lien IP).
Portage C++ de la version Python ;

## Structure

```
CMakeLists.txt
.env.example            -> copier en .env
src/
  main.cpp              orchestrateur (modes détection / --calibrate)
  camera/CSICamera      capture GStreamer (nvargus) threadée, undistortion, calibration damier
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

- g++ >= 9, CMake >= 3.16
- **OpenCV >= 4.7** avec **GStreamer** et le module `objdetect` (API `cv::aruco::ArucoDetector`).
  Vérifier avec `cv::getBuildInformation()`. Si OpenCV est trop ancien : le recompiler
  (`-DWITH_GSTREAMER=ON`, + `-DWITH_CUDA=ON` pour un CLAHE CUDA).
- Un lien IP entre la Jetson et les robots (Wi-Fi 6E, Wi-Fi 5, Ethernet...).

## Build & lancement

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
cp .env.example .env
./build/remote_compute --calibrate     # une fois : génère calibration.yml
./build/remote_compute                 # détection
```

Clavier en mode détection : taper une trame brute (`1`, `4|B`, `4|Y`) pour simuler un message entrant.

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

Pour ne compiler que la démo avec CMake : `cmake -S . -B build -DBUILD_REMOTE_COMPUTE=OFF`.
La bibliothèque `src/planning/AStar.hpp` est réutilisable telle quelle : il suffit de remplir une `Grid`
(par exemple 1 case = 5 cm, kaplas et robots détectés = cases bloquées) et d'appeler `planning::findPath`.

## Tests

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

- `test_detector` : détection ArUco sur images synthétiques (position, orientation, filtre anti-fausses
  détections, report des kaplas, références masquées, caméra en perspective, bruit et ombre).
- `test_protocol` : format des messages cmd 3 et 5, `MatchState`, `WifiLink` (dont un échange UDP local).

Ils ne nécessitent ni caméra ni Jetson, mais **ne remplacent pas un essai sur le matériel réel**.

## État du projet : ce qui est incomplet

Ce code est une base de travail, **pas une version prête pour un match**.

### À remplacer avant tout usage réel

- **Géométrie de l'arène** (`src/arena/ArenaElements.hpp`) : dimensions, zones, IDs et positions des
  marqueurs de référence sont des valeurs d'exemple. Toutes les positions calculées en dépendent.
- **Format des messages** (`src/net/Protocol.hpp`, `MatchState::buildMsg3`) : cmd 3 et cmd 5 sont
  provisoires, à aligner sur le code du robot (puis mettre à jour `tests/test_protocol.cpp`).
- **IDs des kaplas** (`KAPLA_IDS` dans `.env`) : à vérifier.

### Non validé sur le matériel réel

- Caméra CSI IMX219 : pipeline GStreamer et calibration par damier jamais exécutés sur une Jetson.
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
- **Source d'images alternative** (webcam, fichier vidéo) : absente. Le programme principal exige la
  caméra CSI d'une Jetson ; seuls les tests et la démo A\* tournent sans matériel.
- **Affichage UDP** : encodeur logiciel `x264enc` ; l'encodeur matériel Jetson (`nvv4l2h264enc`) n'est
  pas utilisé.
- **Géométrie de l'arène en dur** dans le code, pas dans un fichier de configuration.

### Limites techniques connues

- **Réseau** : UDP sans acquittement, sans authentification ni chiffrement ; à utiliser sur un réseau
  fermé. Un paquet perdu n'est pas renvoyé (les messages périodiques compensent).
- **Threads** : les handlers réseau s'exécutent dans le thread de réception (à garder courts) ; le thread
  clavier de `main.cpp` est détaché (outil de test, à retirer pour un match).
- **Durée du match** : la fin à 100 s et la pré-fin à 90 s ne sont pas testées automatiquement
  (horloge non injectable dans `MatchState`).
- **Intégration continue** : aucune ; OpenCV >= 4.7 doit être compilé pour construire le projet.
