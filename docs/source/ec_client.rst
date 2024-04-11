.. _EtherCAT Client:


***************
EtherCAT Client
***************

.. _EtherCAT Client Architecture:

EtherCAT Client Architecture
=============================

.. image:: _static/EtherCAT_Client_Img/EtherCAT_Client_Img_0.png

.. image:: _static/EtherCAT_Client_Img/EtherCAT_Client_Img_1.png

.. _Code documentation:

Code documentation
=====================================

It's possible to generate the doxygen documentation to read the code of the EtherCAT client::

   doxygen Doxyfile

`Link to the documentation <https://advanced-robotics-facility.github.io/ecat-client-advr/doxygen/>`__ 


Slave Configuration files
--------------------------------------------

The suggestion is to create a directory with this sub-directories:

.. image:: _static/EtherCAT_Client_Img/EtherCAT_Client_Img_2.png


* **joint_config:** Here it's possible to save different slaves configuration files. 
* **joint_map:** File of the slave map (id--->name).
* **"robot_file":** This is an optional file where there are the links to urdf,srdf, model_type and is_model_floating_base information. If this file is present and the user wants to use it, it's better to save the main directory into a workspace where it's possible to find the urdf and srdf.

Motor Setup
--------------------------------------------





