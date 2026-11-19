#!/usr/bin/env bash
# Compile et installe OpenCV 4.8 (avec GStreamer) dans /usr/local sur une Jetson Nano P3450
# (JetPack 4.6.x / Ubuntu 18.04). L'OpenCV 4.1.1 de JetPack reste installé et n'est pas utilisé :
# le CMakeLists.txt du projet exige OpenCV >= 4.7 et trouve donc celui de /usr/local.
#
# Inutile sur Jetson Orin (JetPack 6 fournit déjà OpenCV 4.8 avec GStreamer).
#
# Usage (sur la Jetson) :
#   bash tools/install_opencv_jetson_nano.sh            # ~2 à 3 h, sans CUDA
#   WITH_CUDA=1 bash tools/install_opencv_jetson_nano.sh  # ajoute les modules CUDA (~6 à 8 h)
#
# Sans Ethernet ni Wi-Fi sur la Jetson : Internet passe par le PC via le câble USB et un proxy SOCKS.
#   Sur le PC   : ssh -R 1080 <utilisateur>@192.168.55.1      (laisser cette session ouverte)
#   Sur la Jetson : PROXY=socks5h://127.0.0.1:1080 bash tools/install_opencv_jetson_nano.sh
# On peut aussi copier opencv-4.8.0.zip (et opencv_contrib-4.8.0.zip) dans $WORKDIR : ils ne seront
# alors pas téléchargés.
#
# Conseils : alimentation jack 5 V 4 A (cavalier J48) + mode 10 W (sudo nvpmodel -m 0 ; sudo jetson_clocks),
# ventilateur sur le radiateur, et lancer dans tmux/screen si la connexion SSH peut couper.
set -euo pipefail

OPENCV_VERSION="${OPENCV_VERSION:-4.8.0}"
WITH_CUDA="${WITH_CUDA:-0}"
JOBS="${JOBS:-4}"
WORKDIR="${WORKDIR:-$HOME/opencv_build}"
PROXY="${PROXY:-}"   # ex. socks5h://127.0.0.1:1080 (voir plus haut)

APT_OPTS=()
if [[ -n "$PROXY" ]]; then
    APT_OPTS=(-o "Acquire::http::Proxy=$PROXY" -o "Acquire::https::Proxy=$PROXY")
fi

# Télécharge $1 dans le fichier $2, sauf s'il est déjà là (copié depuis le PC par exemple).
fetch() {
    [[ -s "$2" ]] && { echo "$2 déjà présent"; return; }
    if [[ -n "$PROXY" ]]; then
        curl -fL --proxy "$PROXY" -o "$2.part" "$1"
    else
        wget -q -O "$2.part" "$1"
    fi
    mv "$2.part" "$2"
}

if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "Ce script est prévu pour la Jetson (aarch64)." >&2
    exit 1
fi

echo "=== 1/5 Swap (4 Go) : la compilation dépasse les 4 Go de RAM de la Nano ==="
if ! swapon --show | grep -q /swapfile; then
    sudo fallocate -l 4G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    grep -q '^/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
fi
free -h

echo "=== 2/5 Dépendances ==="
sudo apt-get "${APT_OPTS[@]}" update
sudo apt-get "${APT_OPTS[@]}" install -y build-essential cmake git pkg-config wget curl unzip \
    libjpeg-dev libpng-dev libtiff-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-x \
    libv4l-dev v4l-utils

echo "=== 3/5 Sources OpenCV ${OPENCV_VERSION} ==="
mkdir -p "$WORKDIR"
cd "$WORKDIR"
[[ -d opencv-${OPENCV_VERSION} ]] || {
    fetch "https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip" "opencv-${OPENCV_VERSION}.zip"
    unzip -q "opencv-${OPENCV_VERSION}.zip"
}

# Modules utilisés par le projet (+ dépendances) : compiler le strict nécessaire divise le temps par ~2.
MODULES="core,imgproc,imgcodecs,videoio,calib3d,features2d,flann,objdetect"
CUDA_FLAGS=(-DWITH_CUDA=OFF)
if [[ "$WITH_CUDA" == "1" ]]; then
    [[ -d opencv_contrib-${OPENCV_VERSION} ]] || {
        fetch "https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip" \
              "opencv_contrib-${OPENCV_VERSION}.zip"
        unzip -q "opencv_contrib-${OPENCV_VERSION}.zip"
    }
    MODULES="${MODULES},cudev,cudaarithm,cudaimgproc,cudafilters,cudawarping"
    CUDA_FLAGS=(-DWITH_CUDA=ON -DCUDA_ARCH_BIN=5.3 -DCUDA_ARCH_PTX= -DWITH_CUDNN=OFF
                -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda
                -DOPENCV_EXTRA_MODULES_PATH="$WORKDIR/opencv_contrib-${OPENCV_VERSION}/modules")
fi

echo "=== 4/5 Configuration + compilation (longue) ==="
mkdir -p "opencv-${OPENCV_VERSION}/build"
cd "opencv-${OPENCV_VERSION}/build"
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_LIST="$MODULES" \
    -DWITH_GSTREAMER=ON \
    -DWITH_V4L=ON \
    -DWITH_GTK=OFF -DWITH_QT=OFF \
    -DWITH_FFMPEG=OFF \
    -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF \
    -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF -DBUILD_JAVA=OFF \
    -DENABLE_NEON=ON \
    "${CUDA_FLAGS[@]}"

make -j"$JOBS"

echo "=== 5/5 Installation ==="
sudo make install
sudo ldconfig

echo
echo "OpenCV installé dans /usr/local :"
grep -m1 "CV_VERSION_MINOR" /usr/local/include/opencv4/opencv2/core/version.hpp
echo "Vérifier la ligne 'GStreamer: YES' dans le résumé de configuration ci-dessus."
echo "Ensuite : cd <projet> && rm -rf build && mkdir build && cd build && cmake .. && make -j4"
