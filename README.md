# ecat-client-advr
EtherCAT Client for the the communication with the master: https://advanced-robotics-facility.github.io/ecat-client-advr/

## Installation

Download from this site the last version of EtherCAT Client Debian Package and install it:

https://github.com/Advanced-Robotics-Facility/ecat-client-advr/releases

```
e.g
sudo dpkg -i .....
```

## Dependecies and Packages

### GIT
***********************************************************************************

```
sudo apt update
sudo apt install -y git gitg git-gui
git --version
```
***********************************************************************************
### CMAKE
***********************************************************************************
```
sudo apt install -y file build-essential curl cmake cmake-curses-gui
```
***********************************************************************************

### MATLOGGER
***********************************************************************************
```
sudo apt install -y libmatio-dev libfmt-dev libgtest-dev python3-pip python3-pybind11
git clone https://github.com/ADVRHumanoids/MatLogger2  (actual branch: v1.5.0)
```
***********************************************************************************

### MSGPACK
***********************************************************************************
```
sudo apt install libmsgpack-dev
```
***********************************************************************************

### BOOST
***********************************************************************************
```
sudo apt-get install libboost-system-dev
```
***********************************************************************************

### YAML
***********************************************************************************
```
sudo apt install libyaml-cpp-dev
```
***********************************************************************************

### QT and GUI Tools
***********************************************************************************
```
sudo apt install -y  qttools5-dev libqt5charts5-dev qtdeclarative5-dev libtiff-dev uuid-dev libcurl4-openssl-dev
```
***********************************************************************************

### BlockFactory
***********************************************************************************
