#include "slave_widget.h"

#include <sensor_msgs/JointState.h>

#include "ec_srvs/PrintMotorInfo.h"
#include <iostream>

using namespace XBot::Ui;

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    QUiLoader loader;

    QFile file(":/slave_widget.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;

}

}


SlaveWidget::SlaveWidget ()
{

}

bool SlaveWidget::init(CustomQtWidget::Args& args)
{
    //std::cout << "ClassName: " << this->staticMetaObject.className() << std::endl;

    if(!CustomQtWidget::init(args))
    {
        return false;
    }

    /* Check of XBotCore node presence */
    XBotCoreNodePresence();

    _load_success=false;

    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    auto * layout = new QVBoxLayout;

    /* get Slave Information (joint name, gains, current, etc...) using ROS Service */
    _slaves_descr_vector=getSlavesInfo();

    if(_slaves_descr_vector.empty())
    {
        throw std::runtime_error("Error: Cannot get slaves information");
        return false;
    }


    /* Fill chain selector and widget stack */
    _chain_sel_group =findChild<QGroupBox *>("ChainSel");
    _chain_select = findChild<QComboBox *>("chainSelector");
    _wid_stack  = findChild<QStackedWidget *>("stackedWidget");
    _wid_stack2 = findChild<QStackedWidget *>("stackedWidget2");
    _wid_stack3 = findChild<QStackedWidget *>("stackedWidget3");

    _chain_sel_group->setEnabled(false);

    /* initialization of chain number */
    _ch_numb=0;

    /* Try to contruct all slider widgets */
    try_construct();

    _wid_main_tab=findChild<QTabWidget *>("TabArea");
    _msgTxt = _wid_main_tab->findChild<QTextEdit *>("msgTxt");

    /*  EtherCAT Master commands */
    _fieldtype_combobox = findChild<QComboBox *>("SelectFieldComboBox");
    _fieldtype_combobox->addItem("Start motors");
    _fieldtype_combobox->addItem("Stop motors");
    _fieldtype_combobox->addItem("SDO Information");
    _fieldtype_combobox->addItem("Gains");
    _fieldtype_combobox->addItem("Slaves limits");
    _fieldtype_combobox->addItem("Current");
    _fieldtype_combobox->addItem("Firmware");
    _fieldtype_combobox->addItem("Motor Control");
    _fieldtype_combobox->addItem("Trajectory");

    /* connection of read command function */
    connect(_fieldtype_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(readCommand())
    );

    /*  create mode type for the gains */
    _mode_type_combobox = findChild<QComboBox *>("ModeType");
    _mode_type_combobox->addItem("Position");
    _mode_type_combobox->addItem("Velocity");
    _mode_type_combobox->addItem("Impedance");

    /* connection of read mode type function */
    connect(_mode_type_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(readModeType())
    );

    _motor_ctrl_combobox = findChild<QComboBox *>("MotorCtrlType");
    _motor_ctrl_combobox->addItem("Position");
    _motor_ctrl_combobox->addItem("Velocity");
    _motor_ctrl_combobox->addItem("Torque");
    _motor_ctrl_combobox->addItem("Amperage");
    _motor_ctrl_combobox->addItem("Homing Position");
    _motor_ctrl_combobox->addItem("LED");
    _motor_ctrl_combobox->addItem("FAN");

    /* connection of motor control type function */
    connect(_motor_ctrl_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(readMotorCtrlType())
    );

    _trj_combobox = findChild<QComboBox *>("TrjType");
    _trj_combobox->addItem("Homing");
    _trj_combobox->addItem("Sine");
    _trj_combobox->addItem("Smooth");

    /* connection of trajectory type function */
    connect(_trj_combobox, SIGNAL(currentIndexChanged(int)),this,
        SLOT(readTrjType())
    );



    /****** SET BUTTON and Connection *******/
    _setbtn = findChild<QRadioButton *>("SetBtn");

    if(!_setbtn)
    {
        throw std::runtime_error("Set radio button not exists");
    }

    connect(_setbtn,&QRadioButton::released,
            this, &SlaveWidget::onSetCmdReleased);

    _setbtn->setAutoExclusive(false);

    /****** GET BUTTON and Connection *******/
    _getbtn = findChild<QRadioButton *>("GetBtn");

    if(!_getbtn)
    {
        throw std::runtime_error("get radio button not exists");
    }

    connect(_getbtn,&QRadioButton::released,
            this, &SlaveWidget::onGetCmdReleased);

    _getbtn->setAutoExclusive(false);

    // create joint slave cmd widget
    _slave_cmd_wid  = new SlaveCmdWidget(_slaves_descr_vector,
                                         this);

    // set ec client node with or without XBotCore sub name
    _slave_cmd_wid->set_ec_client_node(_ec_client_node);

    // set chain number
    _slave_cmd_wid->set_chain_number(_ch_numb);

    // set message server text box
    _slave_cmd_wid->set_msgText(_msgTxt);

    // get apply button
    _applybtn = _slave_cmd_wid->get_apply_btn();

    connect(_applybtn, &QPushButton::released,
            this, &SlaveWidget::onApplyCmdReleased);

    // Remove the initial tab widget at zero position inserting the a new one.
    _wid_main_tab->removeTab(0);
    //_wid_main_tab->insertTab(0,_slave_cmd_wid,"Slave Selection");
    _wid_main_tab->setCurrentIndex(0);

    /* Connect chain selector to stacked layout */
    void (QComboBox::* activated_int)(int) = &QComboBox::currentIndexChanged;
    connect(_chain_select, activated_int,
            _wid_stack, &QStackedWidget::setCurrentIndex
           );

    connect(_chain_select, activated_int,
            _wid_stack2, &QStackedWidget::setCurrentIndex
           );

    connect(_chain_select, activated_int,
            _wid_stack3, &QStackedWidget::setCurrentIndex
           );

    /* Connect reset button */
    auto reloadbtn=findChild<QPushButton *>("reloadButton");

    reloadbtn->hide();

    connect(reloadbtn, &QPushButton::released,
            this, &SlaveWidget::on_reload);

    /* Getting command manager (Save Reset Restore) */
    _flash_cmd_manager = findChild<QDialogButtonBox *>("FlashCmd");

    if(!_flash_cmd_manager)
    {
        throw std::runtime_error("_flash_cmd_manager");
    }


   _savebtn = _flash_cmd_manager->button(QDialogButtonBox::Save);

   connect(_savebtn, &QPushButton::released,
           this, &SlaveWidget::onSaveCmdReleased);

   _resetbtn = _flash_cmd_manager->button(QDialogButtonBox::Reset);

   connect(_resetbtn, &QPushButton::released,
           this, &SlaveWidget::onResetCmdReleased);

   _restorebtn = _flash_cmd_manager->button(QDialogButtonBox::RestoreDefaults);

   connect(_restorebtn, &QPushButton::released,
           this, &SlaveWidget::onRestoreCmdReleased);

   _flash_cmd_manager->setEnabled(false);


   /* Getting command manager (Save Reset Restore) */
   _trj_cmd_manager = findChild<QDialogButtonBox *>("TrjCmd");

   if(!_trj_cmd_manager)
   {
       throw std::runtime_error("_flash_cmd_manager");
   }

  _applytbtn = _trj_cmd_manager->button(QDialogButtonBox::Ok);

  connect(_applytbtn, &QPushButton::released,
          this, &SlaveWidget::onStartCmdReleased);

  _cancelbtn = _trj_cmd_manager->button(QDialogButtonBox::Cancel);

  connect(_cancelbtn, &QPushButton::released,
          this, &SlaveWidget::onClearCmdReleased);




    /* disabled and hide some components with xbocore node presence */
    if(_xbotcorenodepresent)
    {
        _fieldtype_combobox->removeItem(0);
        _fieldtype_combobox->removeItem(6);
        _fieldtype_combobox->removeItem(6);
        _chain_sel_group->hide();
        reloadbtn->hide();

        _wid_main_tab->removeTab(0);
        _wid_main_tab->removeTab(0);
        _wid_main_tab->removeTab(0);

        _slave_cmd_wid->HideFirmwareComponents();
    }

    // set up the slave widget
    readCommand();

    auto main_splitter = new QSplitter;
    main_splitter->setOrientation(Qt::Orientation::Horizontal);

    // left vertical layout
    auto lsplitter = new QSplitter;
    lsplitter->setOrientation(Qt::Orientation::Vertical);

    lsplitter->addWidget(_slave_cmd_wid);

    main_splitter->addWidget(lsplitter);

    // right vertical layout
    auto rsplitter = new QSplitter;
    rsplitter->setOrientation(Qt::Orientation::Vertical);

    rsplitter->addWidget(ui);

    main_splitter->addWidget(rsplitter);


    main_splitter->setSizes(QList<int>() << 100 << 1000);

    layout->addWidget(main_splitter);
    setLayout(layout);

    return true;

}

void SlaveWidget::update()
{
}

QString SlaveWidget::name()
{
    return "Ecat";
}

SlaveWidget::~SlaveWidget()
{

}

void SlaveWidget::onSaveCmdReleased()
{
    _slave_cmd_wid->ApplyFlashCmd("Save");
}
void SlaveWidget::onResetCmdReleased()
{
    _slave_cmd_wid->ApplyFlashCmd("Load");
}
void SlaveWidget::onRestoreCmdReleased()
{
    _slave_cmd_wid->ApplyFlashCmd("Default");
}

void SlaveWidget::onStartCmdReleased()
{
    _slave_cmd_wid->ApplyTrjCmd("Start");
}
void SlaveWidget::onClearCmdReleased()
{
    _slave_cmd_wid->ApplyTrjCmd("Clear");
}

void SlaveWidget::onSetCmdReleased()
{
    _slave_cmd_wid->ClearTxtReleased();
    _getbtn->setChecked(false);
    _wid_stack->setEnabled(false);
    _chain_sel_group->setEnabled(false);

    if(SlaveWidget::getFieldType() == "Firmware")
    {
        _slave_cmd_wid->EnableFirmwareComponents();
        _slave_cmd_type=SET_FIRM_VER;
    }

    if((SlaveWidget::getFieldType() == "Start motors") ||
       (SlaveWidget::getFieldType() == "Gains")        ||
       (SlaveWidget::getFieldType() == "Current")      ||
       (SlaveWidget::getFieldType() =="Slaves limits"))
    {
        if(SlaveWidget::getFieldType() == "Current")
        {
            setAmpereTab();
        }
        else if (SlaveWidget::getFieldType() =="Slaves limits")
        {
            setLimitsTab();
        }
        else
        {
            _mode_type_combobox->setVisible(true);
            readModeType();
        }

        if(_setbtn->isChecked())
        {
            _wid_stack->setEnabled(true);
            _chain_sel_group->setEnabled(true);
        }
    }
    if ((SlaveWidget::getFieldType() =="Motor Control")||
        (SlaveWidget::getFieldType() == "Trajectory"))
    {

        _getbtn->setChecked(false);
        _setbtn->setChecked(true);
        _chain_sel_group->setEnabled(true);
    }
}

void SlaveWidget::onGetCmdReleased()
{
    _slave_cmd_wid->ClearTxtReleased();

    _setbtn->setChecked(false);
    _wid_stack->setEnabled(false);
    _chain_sel_group->setEnabled(false);

    if(SlaveWidget::getFieldType() == "Gains")
    {
        _getbtn->setChecked(true);
        _mode_type_combobox->setVisible(false);
        _slave_cmd_type=GET_SLAVE_GAIN_INFO;
    }

    if(SlaveWidget::getFieldType() == "Slaves limits")
    {
        _getbtn->setChecked(true);
        _slave_cmd_type=GET_SLAVE_LIM_INFO;
    }

    if(SlaveWidget::getFieldType() == "Current")
    {
        _getbtn->setChecked(true);
         _slave_cmd_type=GET_SLAVE_AMP_INFO;
    }

    if(SlaveWidget::getFieldType() == "Firmware")
    {
        _getbtn->setChecked(true);
        _slave_cmd_wid->DisableFirmwareComponents();
        _slave_cmd_type=GET_SLAVE_FIRM_VER;
    }

    if(SlaveWidget::getFieldType() == "SDO Information")
    {
        _getbtn->setChecked(true);
        _slave_cmd_type=GET_SDO_INFO;
    }

}

void SlaveWidget::readCommand()
{
    _slave_cmd_wid->ClearTxtReleased();
    _slave_cmd_wid->DisableFirmwareComponents();

    _setbtn->setAutoExclusive(false);
    _getbtn->setAutoExclusive(false);
    _setbtn->setChecked(false);
    _getbtn->setChecked(false);
    _chain_sel_group->setEnabled(false);
    _wid_stack->setEnabled(false);

    readModeType();

    if(!_xbotcorenodepresent)
    {
        _wid_main_tab->setTabEnabled(0,true);
        _wid_main_tab->setTabEnabled(1,false);
        _wid_main_tab->setTabEnabled(2,false);
        _wid_main_tab->setCurrentIndex(0);
    }


    if(SlaveWidget::getFieldType() == "Start motors")
    {
         _getbtn->setDisabled(true);
         _setbtn->setDisabled(false);
         _mode_type_combobox->setVisible(true);
         _motor_ctrl_combobox->setVisible(false);
         _trj_combobox->setVisible(false);
    }
    else if(SlaveWidget::getFieldType() == "Stop motors")
    {
         _getbtn->setDisabled(true);
         _setbtn->setDisabled(true);
         _mode_type_combobox->setVisible(false);
         _motor_ctrl_combobox->setVisible(false);
         _trj_combobox->setVisible(false);
         _slave_cmd_type=STOP_SLAVE;
         if(_xbotcorenodepresent)
            _slave_cmd_wid->onAllCmdReleased();
    }
    else if(SlaveWidget::getFieldType() == "SDO Information")
    {
        _getbtn->setDisabled(false);
        _setbtn->setDisabled(true);

        _setbtn->setAutoExclusive(true);
        _getbtn->setAutoExclusive(true);

        _getbtn->setChecked(true);

         _mode_type_combobox->setVisible(false);
         _motor_ctrl_combobox->setVisible(false);
         _trj_combobox->setVisible(false);
         _slave_cmd_type=GET_SDO_INFO;
    }
    else if(SlaveWidget::getFieldType() == "Gains")
    {
        _getbtn->setDisabled(false);
        _setbtn->setDisabled(false);

        _setbtn->setAutoExclusive(true);
        _getbtn->setAutoExclusive(true);

        _getbtn->setChecked(true);

         _mode_type_combobox->setVisible(false);
         _motor_ctrl_combobox->setVisible(false);
         _trj_combobox->setVisible(false);
         _slave_cmd_type=GET_SLAVE_GAIN_INFO;
    }

    else if(SlaveWidget::getFieldType() == "Slaves limits")
    {
        setLimitsTab();

        if(_internal_repl_sts==REPL_STATUS::OPERATIONAL)
        {
            _getbtn->setDisabled(false);
            _setbtn->setDisabled(true);

            _getbtn->setAutoExclusive(true);

            _getbtn->setChecked(true);
        }
        else
        {
            _getbtn->setDisabled(false);
            _setbtn->setDisabled(false);

            _setbtn->setAutoExclusive(true);
            _getbtn->setAutoExclusive(true);

            _getbtn->setChecked(true);
        }


        _mode_type_combobox->setVisible(false);
        _motor_ctrl_combobox->setVisible(false);
        _trj_combobox->setVisible(false);

         _slave_cmd_type=GET_SLAVE_LIM_INFO;
    }

    else if(SlaveWidget::getFieldType() == "Current")
    {
        setAmpereTab();

        if(_internal_repl_sts==REPL_STATUS::OPERATIONAL)
        {
            _getbtn->setDisabled(false);
            _setbtn->setDisabled(true);

            _getbtn->setAutoExclusive(true);

            _getbtn->setChecked(true);
        }
        else
        {
            _getbtn->setDisabled(false);
            _setbtn->setDisabled(false);

            _setbtn->setAutoExclusive(true);
            _getbtn->setAutoExclusive(true);

            _getbtn->setChecked(true);
        }

        _mode_type_combobox->setVisible(false);
        _motor_ctrl_combobox->setVisible(false);
        _trj_combobox->setVisible(false);

         _slave_cmd_type=GET_SLAVE_AMP_INFO;
    }

    else if(SlaveWidget::getFieldType() == "Firmware")
    {
        if(_internal_repl_sts==REPL_STATUS::OPERATIONAL)
        {
            _getbtn->setDisabled(false);
            _setbtn->setDisabled(true);

            _getbtn->setAutoExclusive(true);

            _getbtn->setChecked(true);
        }
        else
        {
            _getbtn->setDisabled(false);
            _setbtn->setDisabled(false);

            _setbtn->setAutoExclusive(true);
            _getbtn->setAutoExclusive(true);

            _getbtn->setChecked(true);
        }

        _mode_type_combobox->setVisible(false);
        _motor_ctrl_combobox->setVisible(false);
        _trj_combobox->setVisible(false);

         _slave_cmd_type=GET_SLAVE_FIRM_VER;
    }
    else if(SlaveWidget::getFieldType() == "Motor Control")
    {
        readMotorCtrlType();

        _getbtn->setDisabled(true);
        _setbtn->setDisabled(false);

        _getbtn->setAutoExclusive(true);

        _setbtn->setChecked(true);

        _wid_main_tab->setTabEnabled(0,false);
        _wid_main_tab->setTabEnabled(1,false);
        _wid_main_tab->setTabEnabled(2,true);
        _wid_main_tab->setCurrentIndex(2);

        _mode_type_combobox->setVisible(false);
        _motor_ctrl_combobox->setVisible(true);
        _trj_combobox->setVisible(false);

        _chain_sel_group->setEnabled(true);
    }
    else if(SlaveWidget::getFieldType() == "Trajectory")
    {
        readTrjType();

        _getbtn->setDisabled(true);
        _setbtn->setDisabled(false);

        _getbtn->setAutoExclusive(true);

        _setbtn->setChecked(true);

        _wid_main_tab->setTabEnabled(0,false);
        _wid_main_tab->setTabEnabled(1,true);
        _wid_main_tab->setTabEnabled(2,false);
        _wid_main_tab->setCurrentIndex(1);

        _mode_type_combobox->setVisible(false);
        _motor_ctrl_combobox->setVisible(false);
        _trj_combobox->setVisible(true);

        _chain_sel_group->setEnabled(true);

    }
    else
    {
        throw std::runtime_error("Error: Found not valid EtherCAT Client Command");
    }

    if(_xbotcorenodepresent)
    {
        _getbtn->setAutoExclusive(true);
        _setbtn->setAutoExclusive(true);

        if(SlaveWidget::getFieldType() == "Stop motors")
            _getbtn->setDisabled(true);
        else
            _getbtn->setDisabled(false);

        _setbtn->setDisabled(true);

        _getbtn->setChecked(true);
    }
}


void SlaveWidget::setAmpereTab()
{
    _slave_cmd_wid->ClearTxtReleased();

    _slave_cmd_type=SET_MAX_AMP;

    for(int i=0;i<_ch_numb;i++)
    {
      std::string chain="ch_"+std::to_string(i);
      QTabWidget * tab_wid = _tab_wid_map.at(chain);

      for(int k=0;k<3;k++)
      {
          tab_wid->setTabEnabled(k,false);
      }

      tab_wid->setTabEnabled(3,true);
      tab_wid->setCurrentIndex(3);
    }
    _set_type=3;
    _numb_slider=1;
}

void SlaveWidget::setLimitsTab()
{
    _slave_cmd_wid->ClearTxtReleased();

    _slave_cmd_type=SET_LIM;

    for(int i=0;i<_ch_numb;i++)
    {
      std::string chain="ch_"+std::to_string(i);
      QTabWidget * tab_wid = _tab_wid_map.at(chain);

      for(int k=0;k<4;k++)
      {
          tab_wid->setTabEnabled(k,false);
      }

      tab_wid->setTabEnabled(4,true);
      tab_wid->setCurrentIndex(4);
    }
    _set_type=4;
    _numb_slider=4;
}

void SlaveWidget::readModeType()
{
    _slave_cmd_wid->ClearTxtReleased();
    _set_type=0;
    _numb_slider=0;

    for(int i=0;i<_ch_numb;i++)
    {
      std::string chain="ch_"+std::to_string(i);
      auto tab_wid = _tab_wid_map.at(chain);
      for(int k=0;k<5;k++)
      {
          tab_wid->setTabEnabled(k,false);
      }
    }

    if(SlaveWidget::getModeType() == "Position")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab_wid = _tab_wid_map.at(chain);
          tab_wid->setTabEnabled(0,true);
          tab_wid->setCurrentIndex(0);
          _set_type=0;
          _numb_slider=3;
          if(SlaveWidget::getFieldType() == "Gains")
            _slave_cmd_type=SET_POS_GAINS;
          else
            _slave_cmd_type=START_SLAVE_POS;
        }
    }
    else if(SlaveWidget::getModeType() == "Velocity")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab_wid = _tab_wid_map.at(chain);
          tab_wid->setTabEnabled(1,true);
          tab_wid->setCurrentIndex(1);
          _set_type=1;
          _numb_slider=3;
          if(SlaveWidget::getFieldType() == "Gains")
            _slave_cmd_type=SET_VEL_GAINS;
          else
            _slave_cmd_type=START_SLAVE_VEL;
        }
    }
    else if(SlaveWidget::getModeType() == "Impedance")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab_wid = _tab_wid_map.at(chain);
          tab_wid->setTabEnabled(2,true);
          tab_wid->setCurrentIndex(2);
          _set_type=2;
          _numb_slider=5;
          if(SlaveWidget::getFieldType() == "Gains")
            _slave_cmd_type=SET_IMP_GAINS;
          else
            _slave_cmd_type=START_SLAVE_IMP;
        }
    }
    else
    {
        throw std::runtime_error("Error: Found not valid get Mode");
    }
}


void SlaveWidget::readMotorCtrlType()
{
    _set_type=0;
    _numb_slider=0;

    for(int i=0;i<_ch_numb;i++)
    {
      std::string chain="ch_"+std::to_string(i);
      auto tab3_wid = _tab3_wid_map.at(chain);
      for(int k=0;k<7;k++)
      {
          tab3_wid->setTabEnabled(k,false);
      }
    }

    if(SlaveWidget::getMotorCtrlType() == "Position")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab3_wid = _tab3_wid_map.at(chain);
          tab3_wid->setTabEnabled(0,true);
          tab3_wid->setCurrentIndex(0);

          _set_type=0;
          _numb_slider=1;

          _slave_cmd_type=SET_POSITION;
        }
    }
    else if(SlaveWidget::getMotorCtrlType() == "Velocity")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab3_wid = _tab3_wid_map.at(chain);
          tab3_wid->setTabEnabled(1,true);
          tab3_wid->setCurrentIndex(1);

          _set_type=1;
          _numb_slider=1;

          _slave_cmd_type=SET_VELOCITY;
        }
    }
    else if(SlaveWidget::getMotorCtrlType() == "Torque")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab3_wid = _tab3_wid_map.at(chain);
          tab3_wid->setTabEnabled(2,true);
          tab3_wid->setCurrentIndex(2);

          _set_type=2;
          _numb_slider=1;

          _slave_cmd_type=SET_TORQUE;
        }
    }
    else if(SlaveWidget::getMotorCtrlType() == "Amperage")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab3_wid = _tab3_wid_map.at(chain);
          tab3_wid->setTabEnabled(3,true);
          tab3_wid->setCurrentIndex(3);

          _set_type=3;
          _numb_slider=1;

          _slave_cmd_type=SET_AMPERAGE;
        }
    }
    else if(SlaveWidget::getMotorCtrlType() == "Homing Position")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab3_wid = _tab3_wid_map.at(chain);
          tab3_wid->setTabEnabled(4,true);
          tab3_wid->setCurrentIndex(4);

          _set_type=4;
          _numb_slider=1;

          _slave_cmd_type=SET_HOME_POS;
        }
    }
    else if(SlaveWidget::getMotorCtrlType() == "LED")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab3_wid = _tab3_wid_map.at(chain);
          tab3_wid->setTabEnabled(5,true);
          tab3_wid->setCurrentIndex(5);

          _set_type=5;
          _numb_slider=1;

          _slave_cmd_type=SET_LED;
        }
    }
    else if(SlaveWidget::getMotorCtrlType() == "FAN")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab3_wid = _tab3_wid_map.at(chain);
          tab3_wid->setTabEnabled(6,true);
          tab3_wid->setCurrentIndex(6);

          _set_type=6;
          _numb_slider=1;

          _slave_cmd_type=SET_FAN;
        }
    }
    else
    {
        throw std::runtime_error("Error: Found not valid Motor Control Type");
    }
}


void SlaveWidget::readTrjType()
{
    _set_type=0;
    _numb_slider=0;

    for(int i=0;i<_ch_numb;i++)
    {
      std::string chain="ch_"+std::to_string(i);
      auto tab2_wid = _tab2_wid_map.at(chain);
      for(int k=0;k<3;k++)
      {
          tab2_wid->setTabEnabled(k,false);
      }
    }

    if(SlaveWidget::getTrjType() == "Homing")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab2_wid = _tab2_wid_map.at(chain);
          tab2_wid->setTabEnabled(0,true);
          tab2_wid->setCurrentIndex(0);

          _set_type=0;
          _numb_slider=2;

          _slave_cmd_type=SET_HOME_TRJ;
        }
    }
    else if(SlaveWidget::getTrjType() == "Sine")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab2_wid = _tab2_wid_map.at(chain);
          tab2_wid->setTabEnabled(1,true);
          tab2_wid->setCurrentIndex(1);

          _set_type=1;
          _numb_slider=4;

          _slave_cmd_type=SET_SINE_TRJ;
        }
    }
    else if(SlaveWidget::getTrjType() == "Smooth")
    {
        for(int i=0;i<_ch_numb;i++)
        {
          std::string chain="ch_"+std::to_string(i);
          QTabWidget * tab2_wid = _tab2_wid_map.at(chain);
          tab2_wid->setTabEnabled(2,true);
          tab2_wid->setCurrentIndex(2);

          _set_type=2;
          _numb_slider=8;

          _slave_cmd_type=SET_SMOOTH_TRJ;
        }
    }
    else
    {
        throw std::runtime_error("Error: Found not valid Trajectory Type");
    }
}

void SlaveWidget::onApplyCmdReleased()
{
  if(_internal_repl_sts==REPL_STATUS::OPERATIONAL)
  {
    _slave_cmd_wid->set_data_object_type("pdo");
  }
  else
  {
    _slave_cmd_wid->set_data_object_type("sdo");
  }

  bool setting=_setbtn->isChecked();
  if((_slave_cmd_type==STOP_SLAVE) &&
     (_xbotcorenodepresent)        &&
     (_slave_cmd_wid->slave_selected()))
  {
        _slave_cmd_wid->onAllCmdReleased();
  }

  if(SlaveWidget::getFieldType() == "Trajectory")
  {
    _slave_cmd_wid->ApplyTrj(_setup_trj,_slave_cmd_type,setting,_set_type,_numb_slider);
  }
  else if(SlaveWidget::getFieldType() == "Motor Control")
  {
    _slave_cmd_wid->ApplyMotorCtrl(_setup_motor_ctrl,_slave_cmd_type,setting,_set_type,_numb_slider);
  }
  else
  {
    _slave_cmd_wid->ApplyCmd(_setup_slaves,_slave_cmd_type,setting,_set_type,_numb_slider);
  }
}

void SlaveWidget::on_reload()
{
//    auto wid=_wid_main_tab->widget(0);
//    SlaveCmdWidget *slave_cmd_wid;

//    slave_cmd_wid = (SlaveCmdWidget*)wid;

    readCommand();

    try_construct();
}

void SlaveWidget::try_construct()
{
    try
    {
        construct();
    }
    catch(std::exception& e)
    {
        std::string error=e.what();
        throw std::runtime_error("Unable to load widget: "+error);
    }
}

void SlaveWidget::construct()
{
    // remove all widgets
    for(int i = _wid_stack->count() - 1; i >= 0; i--)
    {
        auto * widget = _wid_stack->widget(i);
        _wid_stack->removeWidget(widget);
        widget->deleteLater();
    }
    // construct all widgets 
    std::string chain;
    _ch_numb=0;

    if(!_tab_wid_map.empty())
        _tab_wid_map.clear();

    if(!_tab2_wid_map.empty())
        _tab2_wid_map.clear();

    if(!_tab3_wid_map.empty())
        _tab3_wid_map.clear();

    if(!_setup_slaves.empty())
            _setup_slaves.clear();

    if(!_groups_map.empty())
        _groups_map.clear();

    std::string group="group_";
    int group_id=0,group_count=0;
    int group_size=6;
    std::vector<std::string> groups_vector;

    for(int i=0;i<_slaves_descr_vector.size();i++)
    {
      std::string id_group=group+std::to_string(group_id);
      ec_msgs::SlaveDescription slave_descr=_slaves_descr_vector[i];
      _groups_map[id_group].push_back(slave_descr.slave_name);

      if(group_count==group_size-1)
      {
        groups_vector.push_back(id_group);
        group_id=group_id+1;
        group_count=0;
      }
      else
      {
        group_count++;
      }
      if((group_count>0)&&(i==_slaves_descr_vector.size()-1))   // take last elements that doesn't reach a group 
      {
        groups_vector.push_back(id_group);
        group_id=group_id+1;	
      }
    }

    for(int i=0;i<groups_vector.size();i++)
    {
        auto j_list = _groups_map.at(groups_vector[i]);

        auto tab_wid = new QTabWidget;

        SlidersWidget *_wid_p = new SlidersWidget(j_list,_slaves_descr_vector,"Pos_Gains",this);
        _wid_p->setMaximumHeight(350);

        tab_wid->addTab(_wid_p, "Position");
        _setup_slaves[_ch_numb][0]=_wid_p->get_slider_map();

        auto _wid_v = new SlidersWidget(j_list,_slaves_descr_vector,"Vel_Gains",this);
        _wid_v->setMaximumHeight(350);

        tab_wid->addTab(_wid_v, "Velocity");

        _setup_slaves[_ch_numb][1]=_wid_v->get_slider_map();

        auto _wid_tau = new SlidersWidget(j_list,_slaves_descr_vector,"Imp_Gains",this);

        tab_wid->addTab(_wid_tau, "Effort");
        _setup_slaves[_ch_numb][2]=_wid_tau->get_slider_map();

        auto _wid_a = new SlidersWidget(j_list,_slaves_descr_vector,"Max_Current",this);
        _wid_a->setMaximumHeight(250);

        tab_wid->addTab(_wid_a, "Maximum Current");
        _setup_slaves[_ch_numb][3]=_wid_a->get_slider_map();

        auto _wid_lim = new SlidersWidget(j_list,_slaves_descr_vector,"Limits",this);
        _wid_lim->setMaximumHeight(400);

        tab_wid->addTab(_wid_lim, "Slave Limits");
        _setup_slaves[_ch_numb][4]=_wid_lim->get_slider_map();

        chain="ch_"+std::to_string(_ch_numb);
        _tab_wid_map[chain]=tab_wid;

        _wid_stack->addWidget(tab_wid);

        if(!_load_success)
            _chain_select->addItem(QString::fromStdString(groups_vector[i]));

        _ch_numb++;

        auto tab2_wid = new QTabWidget;

        auto _wid_homing = new SlidersWidget(j_list,_slaves_descr_vector,"Homing",this);
        _wid_homing->setMaximumHeight(250);

        tab2_wid->addTab(_wid_homing, "Homing");
        _setup_trj[_ch_numb][0]=_wid_homing->get_slider_map();

        auto _wid_sine = new SlidersWidget(j_list,_slaves_descr_vector,"Sine",this);
        _wid_sine->setMaximumHeight(400);

        tab2_wid->addTab(_wid_sine, "Sine");
        _setup_trj[_ch_numb][1]=_wid_sine->get_slider_map();

        auto _wid_smooth = new SlidersWidget(j_list,_slaves_descr_vector,"Smooth",this);

        tab2_wid->addTab(_wid_smooth, "Smooth");
        _setup_trj[_ch_numb][2]=_wid_smooth->get_slider_map();

        _tab2_wid_map[chain]=tab2_wid;

        _wid_stack2->addWidget(tab2_wid);


        auto tab3_wid = new QTabWidget;

        auto _wid_position = new SlidersWidget(j_list,_slaves_descr_vector,"Position",this);
        _wid_position->setMaximumHeight(250);

        tab3_wid->addTab(_wid_position, "Position");
        _setup_motor_ctrl[_ch_numb][0]=_wid_position->get_slider_map();

        auto _wid_velocity = new SlidersWidget(j_list,_slaves_descr_vector,"Velocity",this);
        _wid_velocity->setMaximumHeight(250);

        tab3_wid->addTab(_wid_velocity, "Velocity");
        _setup_motor_ctrl[_ch_numb][1]=_wid_velocity->get_slider_map();

        auto _wid_torque = new SlidersWidget(j_list,_slaves_descr_vector,"Torque",this);
        _wid_torque->setMaximumHeight(250);

        tab3_wid->addTab(_wid_torque, "Torque");
        _setup_motor_ctrl[_ch_numb][2]=_wid_torque->get_slider_map();


        auto _wid_amperage = new SlidersWidget(j_list,_slaves_descr_vector,"Current",this);
        _wid_amperage->setMaximumHeight(250);

        tab3_wid->addTab(_wid_amperage, "Amperage");
        _setup_motor_ctrl[_ch_numb][3]=_wid_amperage->get_slider_map();

        auto _wid_home_pos = new SlidersWidget(j_list,_slaves_descr_vector,"Homing_Position",this);
        _wid_home_pos->setMaximumHeight(250);

        tab3_wid->addTab(_wid_home_pos, "Homing Position");
        _setup_motor_ctrl[_ch_numb][4]=_wid_home_pos->get_slider_map();

        auto _wid_led = new SlidersWidget(j_list,_slaves_descr_vector,"LED",this);
        _wid_led->setMaximumHeight(250);

        tab3_wid->addTab(_wid_led, "LED");
        _setup_motor_ctrl[_ch_numb][5]=_wid_led->get_slider_map();


        auto _wid_fan = new SlidersWidget(j_list,_slaves_descr_vector,"FAN",this);
        _wid_fan->setMaximumHeight(250);

        tab3_wid->addTab(_wid_fan, "FAN");
        _setup_motor_ctrl[_ch_numb][6]=_wid_fan->get_slider_map();

        _tab3_wid_map[chain]=tab3_wid;

        _wid_stack3->addWidget(tab3_wid);



    }

    _chain_select->setCurrentIndex(0);
    _load_success = true;
    _wid_stack->setEnabled(false);
}


std::vector<ec_msgs::SlaveDescription> SlaveWidget::getSlavesInfo()
{
    ros::NodeHandle n;
    ros::ServiceClient client=n.serviceClient<ec_srvs::PrintMotorInfo>(_ec_client_node+"/print_motors_info");

    ec_srvs::PrintMotorInfo srv;
    srv.request.slave_name.push_back("all");
    std::vector<ec_msgs::SlaveDescription> slaves_descr_vector;

    if(client.call(srv))
    {
        slaves_descr_vector=srv.response.slaves_descr;
        if(slaves_descr_vector.size()==0)
        {
            std::cout << "Slaves not found!" << std::endl;
        }
    }
    else
    {
        std::cout << "Failed to get the slaves information!" << std::endl;
    }
    return (slaves_descr_vector);
}

void SlaveWidget::XBotCoreNodePresence()
{
    ros::NodeHandle n;
    std::vector< std::string > nodes;
     if(ros::master::getNodes(nodes))
     {
         for (std::vector<std::string>::iterator it = nodes.begin() ; it != nodes.end(); ++it)
         {
             if(*it=="/xbotcore")
             {
                 _xbotcorenodepresent=true;
                 //_ec_client_node="xbotcore/ec_client";
                 return;
             }
         }
     }
}


void SlaveWidget::set_internal_repl_sts(REPL_STATUS sts)
{
  if(_internal_repl_sts!=sts)
  {
    _internal_repl_sts=sts;

    if(_internal_repl_sts==REPL_STATUS::PRE_OPERATIONAL)
    {
        removeOperationalCmd();
        _flash_cmd_manager->setEnabled(true);
    }
    else if(_internal_repl_sts==REPL_STATUS::OPERATIONAL)
    {
        addOperationalCmd();
         _flash_cmd_manager->setEnabled(false);
    }
  }
}

void SlaveWidget::removeOperationalCmd()
{
    if(_fieldtype_combobox->findText("Start motors")>=0)
        _fieldtype_combobox->removeItem(0);

    if(_fieldtype_combobox->findText("Stop motors")>=0)
        _fieldtype_combobox->removeItem(0);

    if(_fieldtype_combobox->findText("Motor Control")>=0)
        _fieldtype_combobox->removeItem(5);

    if(_fieldtype_combobox->findText("Trajectory")>=0)
        _fieldtype_combobox->removeItem(5);

    _fieldtype_combobox->setCurrentIndex(0);
}

void SlaveWidget::addOperationalCmd()
{
    if(_fieldtype_combobox->findText("Start motors")==-1)
        _fieldtype_combobox->insertItem(0,"Start motors");

    if(_fieldtype_combobox->findText("Stop motors")==-1)
        _fieldtype_combobox->insertItem(1,"Stop motors");

    if(_fieldtype_combobox->findText("Motor Control")==-1)
        _fieldtype_combobox->insertItem(6,"Motor Control");

    if(_fieldtype_combobox->findText("Trajectory")==-1)
        _fieldtype_combobox->insertItem(7,"Trajectory");

    _fieldtype_combobox->setCurrentIndex(0);

}

/************** GETTING AREA *************/

std::string SlaveWidget::getFieldType() const
{
    return _fieldtype_combobox->currentText().toStdString();
}

std::string SlaveWidget::getModeType() const
{
    return _mode_type_combobox->currentText().toStdString();
}

std::string SlaveWidget::getMotorCtrlType() const
{
    return _motor_ctrl_combobox->currentText().toStdString();
}

std::string SlaveWidget::getTrjType() const
{
    return _trj_combobox->currentText().toStdString();
}

std::map<int ,std::map<int,std::map<std::string, SliderWidget *>>> SlaveWidget::get_setup_slaves()
{
    return _setup_slaves;
}

QStackedWidget * SlaveWidget::get_stacked_widget()
{
    return _wid_stack;
}

std::unordered_map<std::string,QTabWidget *> SlaveWidget::get_tab_map_widget()
{
    return _tab_wid_map;
}

int SlaveWidget::get_numb_chains()
{
   return _ch_numb;
}

bool SlaveWidget::get_xbotcore_node_presence()
{
    return _xbotcorenodepresent;
}

std::string SlaveWidget::get_ec_client_name()
{
   return _ec_client_node;
}


REPL_STATUS SlaveWidget::get_internal_repl_sts()
{
    return _internal_repl_sts;
}
