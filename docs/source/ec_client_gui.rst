.. _EtherCAT Client GUI:

*********************
EtherCAT Client GUI
*********************

.. image:: _static/EtherCAT_Client_GUI_Img/EtherCAT_Client_GUI_Img_0.png

.. _EtherCAT GUI Architecture:

EtherCAT GUI Architecture
=============================

.. image:: _static/EtherCAT_Client_GUI_Img/EtherCAT_Client_GUI_Img_1.png

The main class of the GUI is the EcGuiStart. This one is responsible to build up all GUI components shown in the picture. 
Two sub-classes are used for different scopes:

* **EcGuiFirmware:** This class is responsible to flash the firmware of the EtherCAT slaves.
* **EcGuiNet:** This class is responsible to create the connection with the EtherCAT Master.
* **EcGuiWrapper:** This class has different components working with EtherCAT Client created inside the EcGuiStart class:

  * **EcGuiCmd:** This class is used to start/stop the controllers for the EtherCAT slaves that can be controlled such as motors,valve,pumps etc....
  * **EcGuiPdo:** This class is used to send PDOs and receiving/plotting them.
  * **EcGuiSdo:** This class is used to read and write SDOs, research them, save and open configuration files (**EcGuiSdoWizard** Class) and use the flash commands. 
  * **EcLogger:** This class is used to log/record the PDOs created in the Ec Client library.

.. _EtherCAT Client GUI Code documentation:

EtherCAT Client GUI Code documentation
========================================

It's possible to generate the doxygen documentation to read the code of the EtherCAT client GUI::

   doxygen Doxyfile

`Link to the documentation <https://advanced-robotics-facility.github.io/ecat-client-advr/gui/>`__ 

.. _EtherCAT Client GUI Network setup:

EtherCAT Client GUI Network setup
========================================

