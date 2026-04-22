#!/bin/bash
set -e

# https://source.denx.de/Xenomai/xenomai/-/wikis/home
# current stable Xenomai release 
XN_VER=3.3.x
XN_DIR=$HOME/xenomai-$XN_VER
    

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
	# Update timestamp
	sudo -nv; sleep 200
	# Exit when the parent process is not running any more.
	kill -0 $$ 2>/dev/null || exit
done &

######################################################################
# Install Dependencies
######################################################################
install_pkgs() {

    echo "### update apt and install some deps" 
    sudo apt update
    sudo apt install -y libelf-dev aptitude apt-file libssl-dev git-core binutils bison flex bc ca-certificates libssl-dev \
    tzdata locales keyboard-configuration psmisc rsync lshw ethtool wget curl unzip ntp less vim bash-completion openssh-server \
    avahi-daemon nfs-common autotools-dev autoconf libfuse-dev cmake libncurses5-dev cmake-curses-gui libcap2-bin uuid-dev build-essential \
    pkg-config usbutils wireless-tools protobuf-compiler libfmt-dev python3-pybind11 gnutls-bin \
    libboost-all-dev libzmq5 libyaml-cpp-dev libprotobuf-dev libeigen3-dev libspdlog-dev libmsgpack-dev libjsoncpp-dev libspnav-dev 
    sudo apt-file update
    sudo chown -R $USER.$USER /usr/local/

}

######################################################################
# Xenomai
######################################################################
setup_xeno() {
    # get sources
    pushd $HOME   
    if [ ! -d xenomai-$XN_VER ]; then
      git clone --branch stable/v$XN_VER --depth 1 https://source.denx.de/Xenomai/xenomai.git xenomai-$XN_VER
    fi
    # compile and install xenomai
    cd xenomai-$XN_VER
    ./scripts/bootstrap
    ./configure --enable-pshared --enable-tls --enable-dlopen-libs --enable-async-cancel --enable-smp
    make -j$(nproc)
    sudo make install
    popd
    
    # get cmake patch and apply it
    pushd $HOME
    if [ ! -d cmake_xenomai ]; then
        git clone https://github.com/nolange/cmake_xenomai.git
    fi
    TARGETDIR=/usr/xenomai/lib/cmake/xenomai
    sudo mkdir -p $TARGETDIR
    cd cmake_xenomai
    sudo ./config/install_cmakeconfig.sh --version $XENOVER -- $TARGETDIR    
    popd
 
    sudo groupadd -f xenomai
    sudo usermod -a -G xenomai $USER
    echo /usr/xenomai/lib/ > xenomai.conf
    sudo cp -f xenomai.conf /etc/ld.so.conf.d/
    sudo ldconfig
    sudo cp -f /usr/xenomai/etc/udev/rules.d/rtdm.rules /etc/udev/rules.d/
    sudo cp -f /usr/xenomai/etc/udev/rules.d/00-rtnet.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    sudo cp ec_xeno3.sh /usr/local/bin
    sudo cp -f xeno.service /lib/systemd/system/
    sudo systemctl enable xeno
    sudo systemctl start xeno || true

    echo 'export PATH=$PATH:/usr/xenomai/bin' | sudo tee /etc/profile.d/xenomai.sh
}

######################################################################
# Kernel
######################################################################
kernel_build() {   
    # KDIR must be set by the caller (kernel_dovetail / kernel_intel_dovetail)
    pushd $KDIR
    make olddefconfig
    make -j$(nproc) bzImage modules 
    sudo make modules_install install
    popd
    sudo cp dflt_grub /etc/default/grub
    sudo update-grub
}

kernel_dovetail() {
    KVER=6.12
    KDIR=$HOME/linux-$KVER-dovetail
    pushd $HOME
    if [ ! -d "${KDIR}" ]; then
        git clone --branch v$KVER.y-dovetail --depth 1 https://source.denx.de/Xenomai/linux-dovetail.git $KDIR
    fi
    if [ ! -d "${XN_DIR}" ]; then
        setup_xeno
    fi
    $XN_DIR/scripts/prepare-kernel.sh --linux=$KDIR --arch=x86
    popd
    # copy config
    cp config-6.12.19-xeno-3.3.2 $KDIR/.config
}

kernel_intel_dovetail() {
    KDIR=$HOME/linux-6.12-intel-dovetail
    pushd $HOME
    if [ ! -d "${KDIR}" ]; then
        git clone --branch 6.12/dovetail-xenomai --depth 1 https://github.com/intel/linux-intel-lts.git $KDIR
    fi
    if [ ! -d "${XN_DIR}" ]; then
        setup_xeno
    fi
    $XN_DIR/scripts/prepare-kernel.sh --linux=$KDIR --arch=x86
    popd
    # copy config
    cp config-6.12.19-xeno-3.3.2 $KDIR/.config
}

######################################################################
######################################################################
######################################################################
if dialog --yesno "Enable Xenomai co-kernel ?" 5 30 ; then
    INFO="Xenomai ON"
    BUILD_DIR=build_rt
    XN=ON
else    
    INFO="Xenomai OFF"
    BUILD_DIR=build
    XN=OFF
fi

CHOICES="$(dialog --stdout --backtitle "${INFO}" \
                  --checklist "Select actions:" 0 0 0 \
  1 "install pkgs" off \
  2 "setup xeno" off \
  3 "kernel dovetail" off \
  4 "kernel intel dovetail" off \
  )"
clear
for CH in ${CHOICES}; do
    case ${CH} in
        1) 
            install_pkgs
            ;;
        2)
            setup_xeno
            ;;
        3)
            kernel_dovetail
            kernel_build
            ;;
        4)
            kernel_intel_dovetail
            kernel_build
            ;;
        *)
            ;;
    esac
done

sudo ldconfig
sudo apt autoremove -y