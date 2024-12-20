#include "ec_gui_pdo.h"
#include "ec_gui_utils.h"

EcGuiPdo::EcGuiPdo(EcGuiSlider::Ptr ec_gui_slider, QWidget *parent):
QWidget(parent),
_ec_gui_slider(ec_gui_slider)
{
    _tree_wid = parent->findChild<QTreeWidget *>("PDO");
    
    _receive_timer= new QElapsedTimer();
    _send_timer= new QElapsedTimer();
    
    _custom_plot = new QCustomPlot();

    // make left and bottom axes transfer their ranges to right and top axes:
    QCustomPlot::connect(_custom_plot->xAxis, SIGNAL(rangeChanged(QCPRange)), _custom_plot->xAxis2, SLOT(setRange(QCPRange)));
    QCustomPlot::connect(_custom_plot->yAxis, SIGNAL(rangeChanged(QCPRange)), _custom_plot->yAxis2, SLOT(setRange(QCPRange)));

    // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
    _custom_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);


    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%s");
    _custom_plot->xAxis->setTicker(timeTicker);
    _custom_plot->xAxis->setLabel("Time [s]");
    _custom_plot->axisRect()->setupFullAxesBox();
    _custom_plot->yAxis->setRange(-100, 100);
    
    auto cplot  = parent->findChild<QVBoxLayout *>("cplot");
    cplot->addWidget(_custom_plot);
    
    _stop_plotting_btn = parent->findChild<QPushButton *>("stopPlotting");

    connect(_stop_plotting_btn, &QPushButton::released,
           this, &EcGuiPdo::onStopPlotting);

    _update_plot=_first_update=_clear_plot=false;
    
    _slider_map=_ec_gui_slider->get_sliders();
    
    _motor_rx_v.resize(MotorPdoRx::pdo_size);
    _motor_tx_v.resize(MotorPdoTx::pdo_size);
    _pow_rx_v.resize(PowPdoRx::pdo_size);
    _ft_rx_v.resize(FtPdoRx::pdo_size);
    _imu_rx_v.resize(ImuPdoRx::pdo_size);
    _valve_rx_v.resize(ValvePdoRx::pdo_size);
    _valve_tx_v.resize(ValvePdoTx::pdo_size);
    _pump_rx_v.resize(PumpPdoRx::pdo_size);
    _pump_tx_v.resize(PumpPdoTx::pdo_size);

    _battery_level = parent->findChild<QLCDNumber *>("BatteryLevel");
    _battery_level->setDigitCount(6);
    _battery_level->display(888888);
    _battery_level->setStyleSheet("background: red; color: #00FF00");
    _v_batt=0.0;
}
      
EcGuiPdo::~EcGuiPdo(){}

/********************************************************* READ PDO***********************************************************************************************/
      
void EcGuiPdo::restart_receive_timer()
{
    _receive_timer->restart();
}

void EcGuiPdo::restart_ec_gui_pdo(EcIface::Ptr client)
{
    _client.reset();
    _client=client;
    
    _tree_wid->clear();
    _custom_plot->clearGraphs();
    _graph_pdo_map.clear();
    _first_update=false;
    _custom_plot->legend->setVisible(false);
    _custom_plot->xAxis->setRange(0, 8, Qt::AlignRight);
    _custom_plot->replot();
}

/************************************* SEARCH SLAVE INTO TREE WID ***************************************/
QTreeWidgetItem * EcGuiPdo::search_slave_into_treewid(std::string esc_id_name)
{
    QTreeWidgetItem * topLevel_read=nullptr;
    for(int i=0;i<_tree_wid->topLevelItemCount();i++)
    {
      topLevel_read =_tree_wid->topLevelItem(i);
      if(esc_id_name==topLevel_read->text(1).toStdString())
      {
          topLevel_read =_tree_wid->topLevelItem(i);
          return(topLevel_read);
      }
    }
    return(nullptr);
}
/************************************* SEARCH SLAVE INTO TREE WID ***************************************/

/************************************* GENERATE COLORS FOR THE GRAPH ***************************************/
void EcGuiPdo::create_color(std::string esc_id_pdo)
{
    auto color= QColor::fromHslF(_currentHue, 1.0, 0.5);
    _currentHue += 0.618033988749895f;
    _currentHue = std::fmod(_currentHue, 1.0f);
    _color_pdo_map[esc_id_pdo] = color;
}
/************************************* GENERATE COLORS FOR THE GRAPH ***************************************/

/************************************* CONVERT PDO NAME***************************************/
QList<QString> EcGuiPdo::get_pdo_fields(const std::vector<std::string> pdo_name)
{
    QList<QString> list;
    list.reserve(pdo_name.size());
    for(const auto &pdo_name_v:pdo_name){
        list.push_back(QString::fromStdString(pdo_name_v));
    }
    return list;
}

/************************************* INITIAL SETUP ***************************************/
QTreeWidgetItem * EcGuiPdo::initial_setup(std::string esc_id_name,QList<QString> pdo_fields)
{
      QTreeWidgetItem * topLevelrtn = new QTreeWidgetItem();
      topLevelrtn->setText(1,QString::fromStdString(esc_id_name));
      topLevelrtn->setText(2,"");
      topLevelrtn->setText(3,"Rx");

      for(const auto &pdo_fields_value:pdo_fields)
      {
          std::string esc_id_pdo = esc_id_name + "_" + pdo_fields_value.toStdString();
          create_color(esc_id_pdo);
          
          QTreeWidgetItem * item = new QTreeWidgetItem();
          item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
          item->setCheckState(1,Qt::Unchecked);
          item->setText(1,pdo_fields_value);
          topLevelrtn->addChild(item);
      }
      
      _tree_wid->addTopLevelItem(topLevelrtn);
      return(topLevelrtn);
}
/************************************* INITIAL SETUP ***************************************/

/************************************* FILL DATA ***************************************/
void EcGuiPdo::fill_data(std::string esc_id_name,QTreeWidgetItem * topLevel,QList<QString> pdo_fields,std::vector<float> pdo)
{
    /************************************* TIME ************************************************/
    topLevel->setText(0,QString::number(_s_receive_time, 'f', 3));
    /************************************* TIME ***********************************************/
    size_t pdo_fields_size = static_cast<size_t>(pdo_fields.size());
    if(pdo_fields_size!=pdo.size()){
        std::cout << "pdo fields size: " << std::to_string(pdo_fields_size) 
                  << " pdo value size: " << std::to_string(pdo.size()) << std::endl;
        throw std::runtime_error("fatal error: on pdo size fields!");
    }

    try{
        /************************************* DATA ***********************************************/
        QString raw_data="";
        int k=0;
        for(const auto &pdo_fields_value:pdo_fields)
        {
            QTreeWidgetItem * item = topLevel->child(k);
            QString data=QString::number(pdo[k], 'f', 6);
            item->setText(2,data);
            raw_data=raw_data+data+ " ";
            
            std::string esc_id_pdo = esc_id_name + "_" + pdo_fields_value.toStdString();
            QCPGraph * graph_pdo=_graph_pdo_map[esc_id_pdo];
            
            if(item->checkState(1)==Qt::Checked)
            {
                if(!graph_pdo)
                {
                    graph_pdo = _custom_plot->addGraph();
                    graph_pdo->setPen(QPen(_color_pdo_map[esc_id_pdo]));
                    graph_pdo->setName(QString::fromStdString(esc_id_pdo));
                    graph_pdo->addToLegend();
                    _graph_pdo_map[esc_id_pdo]=graph_pdo;
                }
                
                // add data to lines:
                graph_pdo->addData(_s_receive_time,data.toDouble());
                //update plot
                _update_plot |= true;
            }
            k++;
        }
        /************************************* DATA ************************************************/

        /************************************* RAW DATA ********************************************/
        topLevel->setText(4,raw_data);
        /************************************* RAW DATA ********************************************/
        
    }catch (const std::out_of_range &oor) {}
}

void EcGuiPdo::read()
{
    /************************************* READ PDOs  ********************************************/
    update_plot();
    read_motor_status();
    read_ft_status();
    read_pow_status();
    read_imu_status();
    read_valve_status();
    read_pump_status();
    /************************************* READ PDOs  ********************************************/

    read_motor_ref();
    read_valve_ref();
    read_pump_ref();
}

void EcGuiPdo::read_motor_status()
{
    _client->get_motor_status(_motor_status_map);
    if(!_motor_status_map.empty()){
        for ( const auto &[esc_id, motor_rx_pdo] : _motor_status_map){
            std::string esc_id_name="motor_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel){
                _motor_pdo_fields=get_pdo_fields(MotorPdoRx::name);
                topLevel= initial_setup(esc_id_name,_motor_pdo_fields);
            }
            if(MotorPdoRx::make_vector_from_tuple(motor_rx_pdo,_motor_rx_v)){
                fill_data(esc_id_name,topLevel,_motor_pdo_fields,_motor_rx_v);
            }
            
            /************************************* ALIGN POSITION SLIDERS with the motor position ********************************************/
            double motor_pos=std::get<2>(motor_rx_pdo);
            if(_slider_map.motor_sw_map.count(esc_id)>0){
                _slider_map.motor_sw_map[esc_id]->set_spinbox_value(1,motor_pos);
            }
            /************************************* ALIGN POSITION SLIDERS with the motor position ********************************************/
        }
        
    }
}

void EcGuiPdo::read_ft_status()
{
    _client->get_ft_status(_ft_status_map);
    /*************************************FT*****************************************************************/
     if(!_ft_status_map.empty()){
        for ( const auto &[esc_id, ft_rx_pdo] : _ft_status_map){
            std::string esc_id_name="ft_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel){
                _ft6_pdo_fields=get_pdo_fields(FtPdoRx::name);
                topLevel= initial_setup(esc_id_name,_ft6_pdo_fields);
            }
            if(FtPdoRx::make_vector_from_tuple(ft_rx_pdo,_ft_rx_v)){
                fill_data(esc_id_name,topLevel,_ft6_pdo_fields,_ft_rx_v);
            }

        }
     }
    /*************************************FT*****************************************************************/
}

inline void EcGuiPdo::read_pow_status()
{
    _client->get_pow_status(_pow_status_map);
    /*************************************Power Board*****************************************************************/
     if(!_pow_status_map.empty()){
        for ( const auto &[esc_id, pow_rx_pdo] : _pow_status_map){
            _battery_level->display(std::get<0>(pow_rx_pdo));
            std::string esc_id_name="pow_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel){
                _pow_pdo_fields=get_pdo_fields(PowPdoRx::name);
                topLevel= initial_setup(esc_id_name,_pow_pdo_fields);
            }
            if(PowPdoRx::make_vector_from_tuple(pow_rx_pdo,_pow_rx_v)){
                fill_data(esc_id_name,topLevel,_pow_pdo_fields,_pow_rx_v);
            }
        }
     }
    /*************************************Power Board*****************************************************************/
}

void EcGuiPdo::read_imu_status()
{
    _client->get_imu_status(_imu_status_map);
    /*************************************IMU*****************************************************************/
     if(!_imu_status_map.empty()){
        for ( const auto &[esc_id, imu_rx_pdo] : _imu_status_map){
            std::string esc_id_name="imu_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel){
                _imu_pdo_fields=get_pdo_fields(ImuPdoRx::name);
                topLevel= initial_setup(esc_id_name,_imu_pdo_fields);
            }
            if(ImuPdoRx::make_vector_from_tuple(imu_rx_pdo,_imu_rx_v)){
                fill_data(esc_id_name,topLevel,_imu_pdo_fields,_imu_rx_v);
            }
        }
     }
    /*************************************IMU*****************************************************************/
}
void EcGuiPdo::read_valve_status()
{
    _client->get_valve_status(_valve_status_map);
    /*************************************VALVE*****************************************************************/
     if(!_valve_status_map.empty()){
        for ( const auto &[esc_id, valve_rx_pdo] : _valve_status_map){
            std::string esc_id_name="valve_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel){
                _valve_pdo_fields=get_pdo_fields(ValvePdoRx::name);
                topLevel= initial_setup(esc_id_name,_valve_pdo_fields);
            }
            if(ValvePdoRx::make_vector_from_tuple(valve_rx_pdo,_valve_rx_v)){
                fill_data(esc_id_name,topLevel,_valve_pdo_fields,_valve_rx_v);
            }
        }
     }
    /*************************************VALVE*****************************************************************/
}

void EcGuiPdo::read_pump_status()
{
    _client->get_pump_status(_pump_status_map);
    /*************************************VALVE*****************************************************************/
     if(!_pump_status_map.empty()){
        for ( const auto &[esc_id, pump_rx_pdo] : _pump_status_map){
            std::string esc_id_name="pump_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel){
                _pump_pdo_fields=get_pdo_fields(PumpPdoRx::name);
                topLevel= initial_setup(esc_id_name,_pump_pdo_fields);
            }
            if(PumpPdoRx::make_vector_from_tuple(pump_rx_pdo,_pump_rx_v)){
                fill_data(esc_id_name,topLevel,_pump_pdo_fields,_pump_rx_v);
            }

            /************************************* ALIGN PUMP SLIDERS with the actual pressure********************************************/
            double pressure=std::get<0>(pump_rx_pdo);
            if(_slider_map.pump_sw_map.count(esc_id)>0){
                _slider_map.pump_sw_map[esc_id]->set_spinbox_value(0,pressure);
            }
            /************************************* ALIGN PUMP SLIDERS with the actual pressure********************************************/
        }
     }
    /*************************************VALVE*****************************************************************/
}

void EcGuiPdo::update_plot()
{
    _ms_receive_time= _receive_timer->elapsed();
    _s_receive_time=(double) _ms_receive_time/1000;
    if(_update_plot)
    {
        if(!_first_update)
        {
            _custom_plot->legend->setVisible(true);
            _first_update=true;
        }
        
        //make key axis range scroll with the data (at a constant range size of 8):
        _custom_plot->xAxis->setRange(_s_receive_time, 8, Qt::AlignRight);
        _custom_plot->replot();
        
        if(_clear_plot)
        {
            _custom_plot->clearGraphs();
            _graph_pdo_map.clear();
            _clear_plot=false;
        }
        
        _update_plot=false;
    }
    else
    {
        if(!_clear_plot && _first_update)
        {
            _clear_plot = true;
        }
    }
}

void EcGuiPdo::onStopPlotting()
{
    for(int i=0;i<_tree_wid->topLevelItemCount();i++)
    {
        auto topLevel =_tree_wid->topLevelItem(i);
        for(int k=0; k< topLevel->childCount(); k++)
        {
            QTreeWidgetItem * item = topLevel->child(k);
            item->setCheckState(1,Qt::Unchecked);
        }
    }
}

/********************************************************* READ PDO***********************************************************************************************/


/********************************************************* WRITE PDO***********************************************************************************************/
void EcGuiPdo::set_filter(int time_ms)
{
    _time_ms = time_ms;
    double ts=((double) _time_ms)/1000;
    _ec_gui_slider->set_sliders_filter(ts);
}

void EcGuiPdo:: set_ctrl_mode(float ctrl_cmd)
{
    _ctrl_cmd = ctrl_cmd;
    _slider_map=_ec_gui_slider->get_sliders(); // read only actual slider widget map.
}

void EcGuiPdo::restart_send_timer()
{
    _send_timer->restart();
}


void EcGuiPdo::write()
{
    _ms_send_time= _send_timer->elapsed();
    _s_send_time=(double) _ms_send_time/1000;

    write_motor_pdo();
    write_valve_pdo();
    write_pump_pdo();
}

void EcGuiPdo::clear_write()
 {
    clear_motor_ref();
    clear_valve_ref();
    clear_pump_ref();
 }

void EcGuiPdo::write_motor_pdo()
{
    bool motors_selected=false;
    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
        motors_selected |=slider_wid->is_slider_enabled();
        break;
    }
    if(!motors_selected){
        return;
    }

    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
        int ctrl_cmd_ref=0x00;
        if(slider_wid->is_slider_enabled()){
            ctrl_cmd_ref=_ctrl_cmd;
        }

        double pos_ref= _slider_map.motor_sw_map[slave_id]->compute_wave(1,_s_send_time);
        double vel_ref= _slider_map.motor_sw_map[slave_id]->compute_wave(2,_s_send_time);
        double tor_ref= _slider_map.motor_sw_map[slave_id]->compute_wave(3,_s_send_time);
        double gain_0= _slider_map.motor_sw_map[slave_id]->compute_wave(4,_s_send_time);
        double gain_1= _slider_map.motor_sw_map[slave_id]->compute_wave(5,_s_send_time);
        double gain_2= _slider_map.motor_sw_map[slave_id]->compute_wave(6,_s_send_time);
        double gain_3= _slider_map.motor_sw_map[slave_id]->compute_wave(7,_s_send_time);
        double gain_4= _slider_map.motor_sw_map[slave_id]->compute_wave(8,_s_send_time);
        uint32_t op= (uint32_t)_slider_map.motor_sw_map[slave_id]->get_spinbox_value(9);
        uint32_t idx= (uint32_t)_slider_map.motor_sw_map[slave_id]->get_spinbox_value(10);
        double aux= _slider_map.motor_sw_map[slave_id]->compute_wave(11,_s_send_time);

        MotorPdoTx::pdo_t   references{ctrl_cmd_ref,
                                       pos_ref, 
                                       vel_ref, 
                                       tor_ref, 
                                       gain_0, 
                                       gain_1,
                                       gain_2, 
                                       gain_3, 
                                       gain_4,
                                       1,
                                       idx,
                                       aux};

        _motors_ref[slave_id]=references;
    }

    if(!_motors_ref.empty()){
       _client->set_motor_reference(_motors_ref);
    }
}

void EcGuiPdo::read_motor_ref()
{
    for ( const auto &[esc_id, references] : _motors_ref){
        if(_slider_map.motor_sw_map[esc_id]->is_slider_enabled()){
            std::string esc_id_name="motor_id_ref_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel){
                _motor_ref_pdo_fields=get_pdo_fields(MotorPdoTx::name);
                topLevel= initial_setup(esc_id_name,_motor_ref_pdo_fields);
            }
            if(MotorPdoTx::make_vector_from_tuple(references,_motor_tx_v)){
                fill_data(esc_id_name,topLevel,_motor_ref_pdo_fields,_motor_tx_v);
            }
        }
    }
}

void EcGuiPdo::clear_motor_ref()
{
    for ( const auto &[esc_id, references] : _motors_ref){
        std::string esc_id_name="motor_id_ref_"+std::to_string(esc_id);
        QTreeWidgetItem *topLevel=search_slave_into_treewid(esc_id_name);
        if(topLevel){
            delete topLevel;
        }
    }

    if(!_motors_ref.empty()){
       _client->set_motor_reference(_motors_ref);
    }
    _motors_ref.clear();
}

void EcGuiPdo::write_valve_pdo()
{
    bool valves_selected=false;
    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
        valves_selected |=slider_wid->is_slider_enabled();
        break;
    }
    if(!valves_selected){
        return;
    }

    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
        double curr_ref=0.0;
        if(slider_wid->is_slider_enabled()){
            curr_ref=0.0;
        }

        curr_ref = _slider_map.valve_sw_map[slave_id]->compute_wave(0,_s_send_time);
        ValvePdoTx::pdo_t   references{curr_ref,
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(1),
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(2),
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(3),
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(4),
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(5),
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(6),
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(7),
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(8),
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(9),
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(10),
                                       _slider_map.valve_sw_map[slave_id]->get_spinbox_value(11)};
        _valves_ref[slave_id]=references;
    }

    if(!_valves_ref.empty()){
       _client->set_valve_reference(_valves_ref);
    }
}

void EcGuiPdo::read_valve_ref()
{
    for ( const auto &[esc_id, references] : _valves_ref){
        if(_slider_map.valve_sw_map[esc_id]->is_slider_enabled()){
            std::string esc_id_name="valve_id_ref_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel){
                _valve_ref_pdo_fields=get_pdo_fields(ValvePdoTx::name);
                topLevel= initial_setup(esc_id_name,_valve_ref_pdo_fields);
            }
            if(ValvePdoTx::make_vector_from_tuple(references,_valve_tx_v)){
                fill_data(esc_id_name,topLevel,_valve_ref_pdo_fields,_valve_tx_v);
            }
        }
    }
}

void EcGuiPdo::clear_valve_ref()
{
    for ( const auto &[esc_id, references] : _valves_ref){
        std::string esc_id_name="valve_id_ref_"+std::to_string(esc_id);
        QTreeWidgetItem *topLevel=search_slave_into_treewid(esc_id_name);
        if(topLevel){
            delete topLevel;
        }
    }

    if(!_valves_ref.empty()){
       _client->set_valve_reference(_valves_ref);
    }
    _valves_ref.clear();
}

void EcGuiPdo::write_pump_pdo()
{
    bool pumps_selected=false;
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
        pumps_selected |=slider_wid->is_slider_enabled();
        break;
    }
    if(!pumps_selected){
        return;
    }

    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
        double press_ref=0.0;
        if(slider_wid->is_slider_enabled()){
            press_ref=0.0;
        }

        press_ref = _slider_map.pump_sw_map[slave_id]->compute_wave(0,_s_send_time);

        PumpPdoTx::pdo_t   references{press_ref,
                                      _slider_map.pump_sw_map[slave_id]->get_spinbox_value(1),
                                      _slider_map.pump_sw_map[slave_id]->get_spinbox_value(2),
                                      _slider_map.pump_sw_map[slave_id]->get_spinbox_value(3),
                                      _slider_map.pump_sw_map[slave_id]->get_spinbox_value(4),
                                      _slider_map.pump_sw_map[slave_id]->get_spinbox_value(5),
                                      _slider_map.pump_sw_map[slave_id]->get_spinbox_value(6),
                                      _slider_map.pump_sw_map[slave_id]->get_spinbox_value(7),
                                      _slider_map.pump_sw_map[slave_id]->get_spinbox_value(8)};

        _pumps_ref[slave_id]=references;
    }

    if(!_pumps_ref.empty()){
       _client->set_pump_reference(_pumps_ref);
    }
}

void EcGuiPdo::read_pump_ref()
{
    for ( const auto &[esc_id, references] : _pumps_ref){
        if(_slider_map.pump_sw_map[esc_id]->is_slider_enabled()){
            std::string esc_id_name="pump_id_ref_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel){
                _pump_ref_pdo_fields=get_pdo_fields(PumpPdoTx::name);
                topLevel= initial_setup(esc_id_name,_pump_ref_pdo_fields);
            }
            if(PumpPdoTx::make_vector_from_tuple(references,_pump_tx_v)){
                fill_data(esc_id_name,topLevel,_pump_ref_pdo_fields,_pump_tx_v);
            }
        }
    }
}


void EcGuiPdo::clear_pump_ref()
{
    for ( const auto &[esc_id, references] : _pumps_ref){
        std::string esc_id_name="pump_id_ref_"+std::to_string(esc_id);
        QTreeWidgetItem *topLevel=search_slave_into_treewid(esc_id_name);
        if(topLevel){
            delete topLevel;
        }
    }

    if(!_pumps_ref.empty()){
       _client->set_pump_reference(_pumps_ref);
    }
    _pumps_ref.clear();
}
/********************************************************* WRITE PDO***********************************************************************************************/
