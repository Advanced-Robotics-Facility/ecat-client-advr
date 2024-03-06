#include "ec_gui_pdo.h"
#include "ec_gui_utils.h"

EcGuiPdo::EcGuiPdo(EcGuiSlider::Ptr ec_gui_slider, QWidget *parent):
QWidget(parent),
_ec_gui_slider(ec_gui_slider)
{
    _tree_wid = parent->findChild<QTreeWidget *>("PDO");
    
    _receive_timer= new QElapsedTimer();
    
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
    
    _imu_rx_v.resize(ImuPdoRx::pdo_size);
    _valve_rx_v.resize(ValvePdoRx::pdo_size);
    _pump_rx_v.resize(PumpPdoRx::pdo_size);
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
    for(int i=0;i<pdo_name.size();i++){
        list.push_back(QString::fromStdString(pdo_name[i]));
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

      for(int index=0; index<pdo_fields.size(); index++)
      {
          std::string esc_id_pdo = esc_id_name + "_" + pdo_fields.at(index).toStdString();
          create_color(esc_id_pdo);
          
          QTreeWidgetItem * item = new QTreeWidgetItem();
          item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
          item->setCheckState(1,Qt::Unchecked);
          item->setText(1,pdo_fields.at(index));
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

    for(int k=0; k < pdo.size();k++)
    {
        try{
            /************************************* DATA ***********************************************/
            QString raw_data="";
            for(int k=0; k<pdo_fields.size(); k++)
            {
                QTreeWidgetItem * item = topLevel->child(k);
                QString data=QString::number(pdo[k], 'f', 6);
                item->setText(2,data);
                raw_data=raw_data+data+ " ";
                
                std::string esc_id_pdo = esc_id_name + "_" + pdo_fields.at(k).toStdString();
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
            }
            /************************************* DATA ************************************************/

            /************************************* RAW DATA ********************************************/
            topLevel->setText(4,raw_data);
            /************************************* RAW DATA ********************************************/
            
        }catch (std::out_of_range oor) {}
    }
}

void EcGuiPdo::read()
{
    /************************************* READ PDOs  ********************************************/
    update_plot();
    read_motor_status();
    read_ft6_status();
    read_pow_status();
    read_imu_status();
    read_valve_status();
    read_pump_status();
    /************************************* READ PDOs  ********************************************/
}

void EcGuiPdo::read_motor_status()
{
    MotorStatusMap motors_status_map;
    _client->get_motors_status(motors_status_map);
    if(!motors_status_map.empty())
    {
        for ( const auto &[esc_id, motor_status] : motors_status_map)
        {
            std::string esc_id_name="motor_id_"+std::to_string(esc_id);
            
            QTreeWidgetItem *topLevel=nullptr;
            topLevel= search_slave_into_treewid(esc_id_name);

            /************************************* INITIAL SETUP ***************************************/
            if(!topLevel)
            {
                topLevel= initial_setup(esc_id_name,_motor_pdo_fields);
            }

            /************************************* INITIAL SETUP ***************************************/

            /************************************* TIME ************************************************/
            topLevel->setText(0,QString::number(_s_receive_time, 'f', 3));
            /************************************* TIME ***********************************************/

            /************************************* DATA ***********************************************/
            try{
                QString raw_data="";
                for(size_t k=0; k<_motor_pdo_fields.size(); k++)
                {
                    QTreeWidgetItem * item = topLevel->child(k);
                    QString data;
                    if(k < 7 || _motor_pdo_fields[k]=="aux")
                    {
                        float value=boost::get<float>(dynamic_get(k,motor_status));
                        data=QString::number(value, 'f', 6);
                        /************************************* ALIGN POSITION SLIDERS with the motor position ********************************************/
                        if(k==1)
                        {
                            double motor_pos=boost::get<float>(dynamic_get(k,motor_status));
                            if(_slider_map.position_sw_map.count(esc_id)>0)
                            {
                                _slider_map.position_sw_map[esc_id]->set_actual_slider_value(motor_pos);
                                _slider_map.position_t_sw_map[esc_id]->set_actual_slider_value(motor_pos);
                            }
                        }
                        /************************************* ALIGN POSITION SLIDERS with the motor position ********************************************/
                    }
                    else
                    {
                        if(_motor_pdo_fields[k]=="Brake_Sts")
                        {
                            uint32_t cmd_aux_sts=boost::get<uint32_t>(dynamic_get(11,motor_status));
                                      
                            uint32_t brake_sts = cmd_aux_sts & 3; //00 unknown
                                                                  //01 release brake 
                                                                  //10 enganged brake
                                                                 //11 error
                            data=QString::number(brake_sts, 'd', 0);
                        }
                        else if(_motor_pdo_fields[k]=="LED_Sts")
                        {
                            uint32_t cmd_aux_sts=boost::get<uint32_t>(dynamic_get(11,motor_status));
                            uint32_t led_sts= (cmd_aux_sts & 4)/4; // 1 or 0 LED  ON/OFF
                            data=QString::number(led_sts, 'd', 0);
                        }
                        else
                        {
                            uint32_t value=boost::get<uint32_t>(dynamic_get(k,motor_status));
                            data=QString::number(value, 'd', 1);
                        }
                    }
                    
                    item->setText(2,data);

                    raw_data=raw_data+data+ " ";
                    
                    if(item->checkState(1)==Qt::Checked)
                    {
                        std::string esc_id_pdo = esc_id_name + "_" + _motor_pdo_fields.at(k).toStdString();
                        QCPGraph * graph_pdo=_graph_pdo_map[esc_id_pdo];
                        
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
                }
                
                /************************************* DATA ************************************************/

                /************************************* RAW DATA ********************************************/
                topLevel->setText(4,raw_data);
                /************************************* RAW DATA ********************************************/
            }catch (std::out_of_range oor) {}
        }
    }
}

void EcGuiPdo::read_ft6_status()
{
    FtStatusMap ft6_status_map;
    _client->get_ft_status(ft6_status_map);
    /*************************************FT*****************************************************************/
     if(!ft6_status_map.empty())
     {
        for ( const auto &[esc_id, ft6_status] : ft6_status_map)
        {
            std::string esc_id_name="ft6_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel)
            {
                topLevel= initial_setup(esc_id_name,_ft6_pdo_fields);
            }
            
            fill_data(esc_id_name,topLevel,_ft6_pdo_fields,ft6_status);
        }
     }
    /*************************************FT*****************************************************************/
}

inline void EcGuiPdo::read_pow_status()
{
    PwrStatusMap pow_status_map;
    _client->get_pow_status(pow_status_map);
    /*************************************Power Board*****************************************************************/
     if(!pow_status_map.empty())
     {
        for ( const auto &[esc_id, pow_status] : pow_status_map)
        {
            std::string esc_id_name="pow_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel)
            {
                topLevel= initial_setup(esc_id_name,_pow_pdo_fields);
            }

            fill_data(esc_id_name,topLevel,_pow_pdo_fields,pow_status);
        }
     }
    /*************************************Power Board*****************************************************************/
}

void EcGuiPdo::read_imu_status()
{
    ImuStatusMap imu_status_map;
    _client->get_imu_status(imu_status_map);
    /*************************************IMU*****************************************************************/
     if(!imu_status_map.empty())
     {
        for ( const auto &[esc_id, imu_rx_pdo] : imu_status_map)
        {
            std::string esc_id_name="imu_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel)
            {
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
     ValveStatusMap valve_status_map;
    _client->get_valve_status(valve_status_map);
    /*************************************VALVE*****************************************************************/
     if(!valve_status_map.empty())
     {
        for ( const auto &[esc_id, valve_rx_pdo] : valve_status_map)
        {
            std::string esc_id_name="valve_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel)
            {
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
     PumpStatusMap pump_status_map;
    _client->get_pump_status(pump_status_map);
    /*************************************VALVE*****************************************************************/
     if(!pump_status_map.empty())
     {
        for ( const auto &[esc_id, pump_rx_pdo] : pump_status_map)
        {
            std::string esc_id_name="pump_id_"+std::to_string(esc_id);
            QTreeWidgetItem *topLevel=nullptr;

            topLevel=search_slave_into_treewid(esc_id_name);
            if(!topLevel)
            {
                _pump_pdo_fields=get_pdo_fields(PumpPdoRx::name);
                topLevel= initial_setup(esc_id_name,_pump_pdo_fields);
            }
            if(PumpPdoRx::make_vector_from_tuple(pump_rx_pdo,_pump_rx_v)){
                fill_data(esc_id_name,topLevel,_pump_pdo_fields,_pump_rx_v);
            }
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
void EcGuiPdo::set_filter(bool first_send, int time_ms)
{
    _first_send = first_send;
    _time_ms = time_ms;
}

void EcGuiPdo:: set_ctrl_mode(float ctrl_cmd)
{
    _ctrl_cmd = ctrl_cmd;
    _slider_map=_ec_gui_slider->get_sliders(); // read only actual slider widget map.
}

double  EcGuiPdo::filtering(SecondOrderFilter<double>::Ptr filter,double actual_value)
{
    if(_first_send)
    {
        filter->reset(actual_value);
        double ts=((double) _time_ms)/1000;
        filter->setTimeStep(ts);
    }

    // Second Order Filtering

    double value_filtered=filter->process(actual_value);

    return value_filtered;
}


void EcGuiPdo::write()
{
    _motors_ref.clear();
    _motor_ref_flags = RefFlags::FLAG_MULTI_REF;
    
    for (auto& [slave_id, slider_wid]:_slider_map.actual_sw_map_selected)
    {
        _gains.clear();
        auto gains_calib_selected=_slider_map.actual_sw_map_selected[slave_id]->get_wid_calibration();

        for(int calib_index=0; calib_index < gains_calib_selected->get_slider_numb(); calib_index++)
        {
            double gain_filtered=filtering(gains_calib_selected->get_slider_filter(calib_index),gains_calib_selected->get_slider_value(calib_index));
            _gains.push_back(gain_filtered);
        }
        if(_ctrl_cmd==0xD4)
        {
            _gains.erase(_gains.begin()+1);
            auto gains_t_calib= _slider_map.torque_sw_map[slave_id]->get_wid_calibration();
            for(int calib_index=0; calib_index < gains_t_calib->get_slider_numb(); calib_index++)
            {
                double gain_t_filtered=filtering(gains_t_calib->get_slider_filter(calib_index),gains_t_calib->get_slider_value(calib_index));
                _gains.push_back(gain_t_filtered);
            }
        }
        else
        {
            _gains.push_back(0);
            _gains.push_back(0);
        }

        double pos_ref= filtering(_slider_map.position_sw_map[slave_id]->get_filer(),_slider_map.position_sw_map[slave_id]->get_spinbox_value());
        if(_ctrl_cmd==0xD4)
        {
            pos_ref= filtering(_slider_map.position_t_sw_map[slave_id]->get_filer(),_slider_map.position_t_sw_map[slave_id]->get_spinbox_value());
        }

        double vel_ref= filtering(_slider_map.velocity_sw_map[slave_id]->get_filer(),_slider_map.velocity_sw_map[slave_id]->get_spinbox_value());
        double tor_ref= filtering(_slider_map.torque_sw_map[slave_id]->get_filer(),_slider_map.torque_sw_map[slave_id]->get_spinbox_value());
                                   
        MotorPdoTx::pdo_t   references{_ctrl_cmd, pos_ref, vel_ref, tor_ref, _gains[0], _gains[1],_gains[2], _gains[3], _gains[4],1,0,0};
        //            ID      CTRL_MODE, POS_REF, VEL_RF, TOR_REF,  GAIN_1,    GAIN_2,   GAIN_3,   GAIN_4,    GAIN_5, OP, IDX,AUX  OP->1 means NO_OP
        _motors_ref[slave_id]=references;

        _first_send=false; // Note: not remove from here, used for all filters.
    }
    if(!_motors_ref.empty())
    {
       _client->set_motors_references(_motor_ref_flags, _motors_ref);
    }
}
/********************************************************* WRITE PDO***********************************************************************************************/
