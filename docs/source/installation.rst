.. _Installation:


***************
Installation
***************

.. _ROS:

ROS
=============================

Follow ROS website steps to install ROS, i.e `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`__.

Install full package::

   sudo apt install ros-melodic-desktop-full

Source into .bashrc ROS packages::

   source /opt/ros/"melodic"/setup.bash


ZMQ
=============================

Install zmq library::

   wget https://github.com/zeromq/libzmq/archive/v4.2.5.zip
   unzip v4.2.5.zip
   cd libzmq-4.2.5
   ./autogen.sh
   ./configure
   make -j8
   sudo make install

Install cppzmq library::

   wget https://github.com/zeromq/cppzmq/archive/v4.2.3.zip
   unzip v4.2.3.zip
   cd cppzmq-4.2.3
   mkdir build
   cd build
   cmake ..
   make
   sudo make install

XBotCore FULL Development
=============================

Install xbotcore full development (choose OS, i.e bionic)::

   wget http://54.73.207.169/nightly/xbot2-full-devel/bionic-nightly.tar.gz
   tar -zxvf bionic-nightly.tar.gz
   cd bionic-18.04-2021_04_13_18_27_38/
   ./install.sh 

Source into .bashrc XBOT packages::

   source /opt/xbot/setup.sh


EtherCAT Dependency Libraries
================================

Install these libraries for the Robot GUI::

   sudo add-apt-repository universe
   sudo apt update
   sudo apt install libfmt-dev
   sudo apt install libqt5charts5-dev

EtherCAT Client Downloading and Buiding
========================================

Prepare a new workspace::

  mkdir "name_workspace"
  cd "name_workspace"
  mkdir src
  cd src
  catkin_init_workspace
  mkdir external
  cd ..
  catkin_make

Source into .bashrc the new workspace::

  source ~/name_workspace/devel/setup.bash 


Download and compile dependency repositories::
  
  cd src/external

  git clone git@github.com:alessiomargan/SOEM.git
  cd SOEM
  git checkout xeno3

  cd ..
  git clone git@github.com:ADVRHumanoids/protobuf.git
  cd protobuf
  git checkout xbot2

  cd ..
  git clone git@gitlab.advr.iit.it:xeno-ecat/ecat_master_advr.git
  cd ecat_master_advr
  git checkout xbot2_fixed

Download and compile EtherCAT Client::

  cd src/external

  git clone git@github.com:ADVRHumanoids/ec_xbot2_client.git


Note: Select the variable to compile (EtherCAT Client, the Real Time Devices for XBotCore2.0 and Robot GUI):

.. image:: _static/EtherCAT_Installation_Img/EtherCAT_Installation_Img_0.png







