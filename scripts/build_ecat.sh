#!/bin/bash
set -e

# Branch versions
SOEM_VERSION=xeno3
MATLOGGER2_VERSION=master
CPPZMQ_VERSION=master
ECAT_MASTER_ADVR_VERSION=feature/cia402_rev
ECAT_CLIENT_ADVR_VERSION=feature/novanta

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if (( $EUID == 0 )); then
    echo "Please do NOT run as root"
    exit
fi

sudo apt install -y dialog

PASSWD="$(dialog --stdout --insecure --passwordbox "Enter ${USER} password:" 0 0)"
clear

echo "### Keep sudo session alive..."
echo "$PASSWD" | sudo -S -v
while true; do
    sudo -nv; sleep 200
    kill -0 $$ 2>/dev/null || exit
done &

######################################################################
# Workspace
######################################################################
WS_NAME="$(dialog --stdout --inputbox "Workspace name:" 0 0 "ecat_ws")"
WS_DIR="$(dialog --stdout --inputbox "Workspace parent directory:" 0 0 "$HOME")"
clear

bash "$SCRIPT_DIR/create_ws.sh" "$WS_NAME" "$WS_DIR"

WS_ROOT="$WS_DIR/$WS_NAME"
INSTALL_PREFIX="$WS_ROOT/install"
SRC_DIR="$WS_ROOT/src"
BUILD_DIR="$WS_ROOT/build"

# source the workspace so CMAKE_PREFIX_PATH etc. are set for this session
source "$WS_ROOT/setup.bash"

######################################################################
# Options
######################################################################
if dialog --yesno "Enable Xenomai Real-Time ?" 5 35; then
    USE_XENO=ON
    INFO="RT (Xenomai ON)"
else
    USE_XENO=OFF
    INFO="Non-RT (Xenomai OFF)"
fi

if dialog --yesno "Build GUI tools ?\n(not needed on headless embedded targets)" 7 45; then
    BUILD_GUI=ON
    INFO="$INFO | GUI ON"
else
    BUILD_GUI=OFF
    INFO="$INFO | GUI OFF"
fi

CHOICES="$(dialog --stdout --backtitle "${INFO}" \
                  --checklist "Select packages to build:" 0 0 0 \
  1 "Install Dependencies" off \
  2 "SOEM" off \
  3 "cppzmq" off \
  4 "MatLogger2" off \
  5 "ecat-master-advr" off \
  6 "ecat-client-advr" off \
  )"
clear

######################################################################
# Install Dependencies
######################################################################
install_pkgs() {
    echo "### Installing Required Dependencies" 
    sudo apt update
    sudo apt install -y git gitg git-gui build-essential curl cmake cmake-curses-gui libgtest-dev \
                        net-tools openssh-server sshpass curl gnome-terminal terminator \
                        libyaml-cpp-dev libfmt-dev \
                        libzmq3-dev protobuf-compiler libmsgpack-dev libboost-system-dev \
                        libeigen3-dev \
                        libmatio-dev python3-pip \
                        qt6-tools-dev qt6-declarative-dev libqt6charts6-dev uuid-dev libtiff-dev qttools5-dev
}

######################################################################
# SOEM
######################################################################
build_soem() {
    echo "### Building SOEM"
    if [ ! -d $SRC_DIR/SOEM ]; then
        git clone -b ${SOEM_VERSION} https://github.com/alessiomargan/SOEM $SRC_DIR/SOEM
    fi
    mkdir -p $BUILD_DIR/SOEM
    cd $BUILD_DIR/SOEM
    cmake $SRC_DIR/SOEM \
        -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
        -DCMAKE_USE_XENOMAI=$USE_XENO
    make -j$(nproc)
    make install
}

######################################################################
# cppzmq
######################################################################
build_cppzmq() {
    echo "### Building cppzmq"
    if [ ! -d $SRC_DIR/cppzmq ]; then
        git clone -b ${CPPZMQ_VERSION} https://github.com/zeromq/cppzmq $SRC_DIR/cppzmq
    fi
    mkdir -p $BUILD_DIR/cppzmq
    cd $BUILD_DIR/cppzmq
    cmake $SRC_DIR/cppzmq \
        -DCMAKE_BUILD_TYPE="Release" \
        -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
        -DCPPZMQ_BUILD_TESTS=OFF
    make -j$(nproc)
    make install
}

######################################################################
# MatLogger2
######################################################################
build_matlogger() {
    echo "### Building MatLogger2"
    sudo apt install -y libmatio-dev python3-pybind11
    if [ ! -d $SRC_DIR/MatLogger2 ]; then
        git clone -b ${MATLOGGER2_VERSION} https://github.com/ADVRHumanoids/MatLogger2 $SRC_DIR/MatLogger2
    fi
    mkdir -p $BUILD_DIR/MatLogger2
    cd $BUILD_DIR/MatLogger2
    cmake $SRC_DIR/MatLogger2 \
        -DCMAKE_BUILD_TYPE="Release" \
        -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
        -DCOMPILE_PY_BINDINGS=OFF
    make -j$(nproc)
    make install
}

######################################################################
# ecat-master-advr
######################################################################
build_ecat_master() {
    echo "### Building ecat-master-advr"
    if [ ! -d $SRC_DIR/ecat-master-advr ]; then
        git clone -b ${ECAT_MASTER_ADVR_VERSION} https://github.com/Advanced-Robotics-Facility/ecat-master-advr $SRC_DIR/ecat-master-advr
    fi
    mkdir -p $BUILD_DIR/ecat-master-advr
    cd $BUILD_DIR/ecat-master-advr
    cmake $SRC_DIR/ecat-master-advr \
        -DCMAKE_BUILD_TYPE="Release" \
        -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
        -DBUILD_UDP_SRV=ON \
        -DENABLE_XENO=$USE_XENO 
    make -j$(nproc)
    touch install_manifest.txt  # workaround: prevent cmake making it root-owned
    make install
}

######################################################################
# ecat-client-advr
######################################################################
build_ecat_client() {
    echo "### Building ecat-client-advr"
    if [ ! -d $SRC_DIR/ecat-client-advr ]; then
        git clone -b ${ECAT_CLIENT_ADVR_VERSION} https://github.com/Advanced-Robotics-Facility/ecat-client-advr $SRC_DIR/ecat-client-advr
    fi

    if [ "$BUILD_GUI" = "ON" ]; then
        sudo apt install -y qt6-tools-dev qt6-declarative-dev libqt6charts6-dev \
            uuid-dev libtiff-dev qttools5-dev
    fi

    mkdir -p $BUILD_DIR/ecat-client-advr
    cd $BUILD_DIR/ecat-client-advr
    cmake $SRC_DIR/ecat-client-advr \
        -DCMAKE_BUILD_TYPE="Release" \
        -DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX \
        -DCMAKE_USE_XENOMAI=$USE_XENO \
        -DENABLE_XENO=$USE_XENO \
        -DCOMPILE_GUI=$BUILD_GUI
    make -j$(nproc)
    make install
}

######################################################################
# Run selected steps
######################################################################
for CH in ${CHOICES}; do
    case ${CH} in
        1) install_pkgs ;;
        2) build_soem ;;
        3) build_cppzmq ;;
        4) build_matlogger ;;
        5) build_ecat_master ;;
        6) build_ecat_client ;;
        *) ;;
    esac
done

######################################################################
sudo ldconfig

# Add workspace and EC_CFG to .bashrc if not already there
if ! grep -q "$WS_ROOT/setup.bash" ~/.bashrc; then
    echo "source $WS_ROOT/setup.bash" >> ~/.bashrc
    echo "### Added workspace to ~/.bashrc: source $WS_ROOT/setup.bash"
fi

EC_CFG_PATH="$SRC_DIR/ecat-client-advr/config/ec_cfg.yaml"
if ! grep -q "EC_CFG" ~/.bashrc; then
    echo "export EC_CFG=$EC_CFG_PATH" >> ~/.bashrc
    echo "### Added EC_CFG to ~/.bashrc"
fi

echo ""
echo "### Done. Run: source ~/.bashrc"

sudo apt autoremove -y