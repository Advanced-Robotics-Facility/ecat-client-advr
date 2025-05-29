.. _WSL:


***********************************
WSL
***********************************

`About Windows Subsystem for Linux <https://learn.microsoft.com/en-us/windows/wsl/about>`__ 

`WSL basic commands <https://learn.microsoft.com/en-us/windows/wsl/basic-commands>`__ 

**Note:** Latest version of Windows has already installed the WSL, if not please follow this preliminary step: 

Install WSL without distribution, since a specific distribution will be delivered::

   wsl --install --no-distribution

Download from this site the latest version of Ubuntu OS for importing on WSL and wslconfig file::

   https://github.com/Advanced-Robotics-Facility/ecat-client-advr/releases
   
**Unzip the Ubuntu-22.04_Facility.zip file and copy Ubuntu-22.04_Facility.tar into C:/%USERNAME%/Desktop/**
   
**Rename and copy the WSL configuration file (.wslconfig) into C:/%USERNAME%/**
   
Import Distro version::

   wsl --import <DistroName> <InstallLocation> <InstallTarFile> 

Example:

* DistroName= Ubuntu-22.04
* InstallLocation= **C:/%USERNAME%/Ubuntu-22.04/** (please create the specific directory).
* InstallTarFile= **/path_of_download/Ubuntu-22.04_Facility.tar**, i.e C:/%USERNAME%/Desktop/Ubuntu-22.04_Facility.tar

::

   wsl --import Ubuntu-22.04 C:/%USERNAME%/Ubuntu-22.04/ C:/%USERNAME%/Desktop/Ubuntu-22.04_Facility.tar

Install Distro version::

   wsl -d <DistroName> -u <UserName> 

Example:

* DistroName= **Ubuntu-22.04**.
* UserName= **user**.

::

   wsl -d Ubuntu-22.04 -u user
  
   
**Other important commands**:

List available Linux distributions::

  wsl --list

WSL Update::

   wsl --update

WSL Shutdown::

   wsl --shutdown

Unregister or uninstall a Linux distribution::

   wsl --unregister <DistributionName>
   
Example:

::

   wsl --unregister Ubuntu-22.04
