.. _Installation:


***************
Installation
***************

RealTime: Embedded
====================

This guide walks through setting up a real-time (RT) embedded environment on a fresh Ubuntu system using Xenomai and an EtherCAT stack.

1. Install Xenomai Kernel
--------------------------

Xenomai provides real-time capabilities by adding a co-kernel to Linux.

Steps::

   sudo apt install -y git
   git clone https://github.com/Advanced-Robotics-Facility/ecat-client-advr.git
   cd ecat-client-advr
   ./scripts/xenomai/prepare.sh

Configuration options:

* Answer **YES** to "Enable Xenomai Real-Time?"
* Select packages:

   * *1:* Dependencies (required)
   * *2:* Xenomai setup (required)
   * *3/4:* Kernel patch (choose based on your CPU)

Then reboot into the Xenomai-patched kernel::

   sudo reboot

After reboot, make sure you're running the Xenomai kernel::

   uname -r

2. Install EtherCAT Stack
--------------------------

Run the build script::

   ./ecat-client-advr/scripts/build_ecat.sh

Configuration options:

   * Enter your workspace name (e.g., code_ws) and parent directory
   * Answer **YES** to "Enable Xenomai Real-Time?"
   * Answer **NO** to "Build GUI tools?" (recommended for embedded systems)
   * Select all packages (and dependencies) for a full installation: 1 2 3 4 5 6

This installs both the EtherCAT master and client libraries.

3. Configure Network Interface
---------------------------------

EtherCAT requires direct control of a network card, bypassing the standard Linux network stack.

Identify your network card::

   lspci -nkv 

Edit configuration script::

   vim /usr/local/bin/ec_xeno3.sh
   # ETH_DRV - set the correct driver for your network card (e.g., e1000e, igb, etc.)
   # Verify REBIND_RT_NICS matches your network card

4. Start Xenomai Service
--------------------------
Restart the Xenomai service to apply everything::

   systemctl restart xeno.service

You can check its status with::

   systemctl status xeno.service

5. Load Environment
-----------------------
Make sure your environment variables are loaded::

   source ~/.bashrc

----------------------------------------------------------------------

Non-RealTime: Embedded/PC
=========================

Same as RealTime Embedded but:

* Skip "Install Xenomai Kernel", "Network Interface Configuration", and "Start Xenomai Service" 
* Answer **NO** to "Enable Xenomai Real-Time?" in *build_ecat.sh*

----------------------------------------------------------------------

Manual Installation
====================

This section describes how to manually install the EtherCAT Client on an **Ubuntu system**. The framework can also be used on Windows via **WSL (Windows Subsystem for Linux)**.

This manual setup replicates the behavior of the provided automated installer script (*build_ecat.sh*).

1. Create the Workspace
-------------------------

Download manually `create_ws.sh` from `ecat-client-advr` repository inside the `scripts` directory and then make it executable::

 chmod +x create_ws.sh

Run the script to generate the workspace structure::

  ./create_ws.sh [name_ws] [dir_ws, default=HOME]
  
Add the generated workspace setup to your shell environment::

  source ~/<workspace_name>/setup.bash 
  
Then use the **src** dir to place all the cloned repositories, and **build** dir (creating the relative package sub-directory (**mkdir package_name**)) for compiling them.

2. Dependencies
--------------------

This section includes all system dependencies required for building and running the stack.

---
**Basic development tools**

::

   sudo apt update
   sudo apt install -y git gitg git-gui
   sudo apt install -y build-essential curl cmake cmake-curses-gui
   sudo apt install -y libgtest-dev

---
**Networking tools**

::

   sudo apt install -y net-tools openssh-server sshpass curl \
                        gnome-terminal terminator

---
**YAML and formatting libraries**

::

   sudo apt install -y libyaml-cpp-dev libfmt-dev

---
**Mechanism Protocols: Zmq, Protobuf, Msgpack, Boost**

::

   sudo apt install -y libzmq3-dev protobuf-compiler libmsgpack-dev libboost-system-dev

---
**Eigen3**

::

   sudo apt install -y libeigen3-dev

---
**MatLogger**

::

   sudo apt install -y libmatio-dev python3-pip

---
**GUI tools (Qt)**

::

   sudo apt install -y qt6-tools-dev qt6-declarative-dev \
                        libqt6charts6-dev uuid-dev libtiff-dev qttools5-dev

---
**Matlab / Simulink block factory (OPTIONAL)**

::

   sudo dpkg --install ecat-client-advr/src/matlab/ec_block/external/blockfactory/(OS)/blockfactory_package-0.8.3-r0.0.1-amd64.deb

3. Clone, Build, and Install Packages
--------------------------------------

Create the build folders::

   cd build/
   mkdir -p SOEM cppzmq MatLogger2 ecat-master-advr ecat-client-advr

Clone the required packages with the correct branches::

   git clone -b xeno3 https://github.com/alessiomargan/SOEM
   git clone -b v4.7.1 https://github.com/zeromq/cppzmq
   git clone -b master https://github.com/ADVRHumanoids/MatLogger2
   git clone -b feature/cia402_rev https://github.com/Advanced-Robotics-Facility/ecat-master-advr
   git clone -b feature/novanta https://github.com/Advanced-Robotics-Facility/ecat-client-advr

**Note:** *ecat-master-advr* is a private repository. Ask to Advanced-Robotics-Facility for the access.

---
**SOEM**

::

   cd build/SOEM
   cmake ../../src/SOEM 
         -DCMAKE_INSTALL_PREFIX=<path-to-install-folder> \
         -DCMAKE_USE_XENOMAI=ON
   make -j$(nproc) install

**Note:** -DCMAKE_USE_XENOMAI=OFF if Non-RealTime

---
**cppzmq**

::

   cd build/cppzmq
   cmake ../../src/cppzmq 
         -DCMAKE_BUILD_TYPE="Release" \
         -DCMAKE_INSTALL_PREFIX=<path-to-install-folder> \
         -DCPPZMQ_BUILD_TESTS=OFF
   make -j$(nproc) install

---
**MatLogger2**

::

   cd build/MatLogger2 
   cmake ../../src/MatLogger2
         -DCMAKE_BUILD_TYPE="Release" \
         -DCMAKE_INSTALL_PREFIX=<path-to-install-folder> \
         -DCOMPILE_PY_BINDINGS=OFF
   make -j$(nproc) install

---
**ecat-master-advr** 

::

   cd build/ecat-master-advr
   cmake ../../src/ecat-master-advr 
         -DCMAKE_BUILD_TYPE="Release" \
         -DCMAKE_INSTALL_PREFIX=<path-to-install-folder>  \
         -DBUILD_UDP_SRV=ON \
         -DENABLE_XENO=ON   
   make -j$(nproc) install

**Note:** -DENABLE_XENO=OFF if Non-RealTime

---
**ecat-client-advr**

::

   cd build/ecat-client-advr
   cmake ../../src/ecat-client-advr 
         -DCMAKE_BUILD_TYPE="Release" \
         -DCMAKE_INSTALL_PREFIX=<path-to-install-folder>  \
         -DCMAKE_USE_XENOMAI=ON \ 
         -DENABLE_XENO=ON \ 
         -DCOMPILE_GUI=OFF                              
   make -j$(nproc) install

**Note1:** -DENABLE_XENO=OFF, -DCMAKE_USE_XENOMAI=OFF if Non-RealTime

**Note2:** -DCOMPILE_GUI=ON if you need the GUI

4. Environmental Variable
-----------------------------

Export environment variable EC_CFG::

 export EC_CFG=~/<workspace_name>/src/ecat-client-advr/config/ec_cfg.yaml

----------------------------------------------------------------------

Install the EtherCAT Client Library from debian package
==========================================================

Download from this site the latest version of EtherCAT Client Debian Package and install it::

   https://github.com/Advanced-Robotics-Facility/ecat-client-advr/releases

Install EtherCAT Client Librarys::

  sudo dpkg --install ecat-client-advr_nrt_package-0.0.1-r0.0.1-amd64.deb