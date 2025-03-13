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
    _custom_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom );// QCP::iSelectPlottables);


    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%s");
    _custom_plot->xAxis->setTicker(timeTicker);
    _custom_plot->xAxis->setLabel("Time [s]");
    _custom_plot->axisRect()->setupFullAxesBox();
    _custom_plot->yAxis->setRange(-100, 100);
    QFont legendFont = font();
    legendFont.setPointSize(12);
    _custom_plot->legend->setFont(legendFont);
    
    auto cplot  = parent->findChild<QVBoxLayout *>("cplot");
    cplot->addWidget(_custom_plot);

    _stop_plotting_btn = parent->findChild<QPushButton *>("stopPlotting");

    connect(_stop_plotting_btn, &QPushButton::released,
           this, &EcGuiPdo::onStopPlotting);

    _update_plot=_first_update=false;

    _battery_level = parent->findChild<QLCDNumber *>("BatteryLevel");
    _battery_level->setDigitCount(4);
    _battery_level->display(888888);
    _battery_level->setStyleSheet("background: red; color: #00FF00");

    _time_pdo=parent->findChild<QLabel *>("TimePdo"); 

    _auto_scroll=parent->findChild<QCheckBox *>("AutoScroll"); 
}
      
EcGuiPdo::~EcGuiPdo()
{

}

/********************************************************* READ PDO***********************************************************************************************/
      
void EcGuiPdo::restart_receive_timer()
{
    _first_update=false;
    _receive_timer->restart();
}

void EcGuiPdo::restart_ec_gui_pdo(EcIface::Ptr client,EcLogger::Ptr ec_logger)
{
    _client=client;
    _ec_logger=ec_logger;
    
    _tree_wid->clear();
    _custom_plot->clearGraphs();
    _graph_pdo_map.clear();
    _first_update=false;
    _custom_plot->legend->setVisible(false);
    _custom_plot->xAxis->setRange(0, 8, Qt::AlignRight);
    _custom_plot->replot();
    _battery_level->display(0.0);
    _time_pdo->setText("0.00");

    _counter_buffer= 0;
    _buffer_size=20;

    _buffer_time.clear();
    _buffer_time.resize(_buffer_size);
    _buffer_pdo_map.clear();
    _pdo_v.clear();
    _esc_pdo_map.clear();

    _slider_map=_ec_gui_slider->get_sliders(); // read only actual slider widget map.
    _motors_selected=_valves_selected=_pumps_selected=false;
    init_write_pdo();

}
/************************************* GENERATE GRAPH ***************************************/
void EcGuiPdo::create_graph(const int &esc_id,const int &index,const std::string &esc_pdo_name)
{
    auto color= QColor::fromHslF(_currentHue, 1.0, 0.5);
    _currentHue += 0.618033988749895f;
    _currentHue = std::fmod(_currentHue, 1.0f);

    QCPGraph * graph_pdo= _custom_plot->addGraph();
    graph_pdo->setPen(QPen(color));
    graph_pdo->setName(QString::fromStdString(esc_pdo_name));
    graph_pdo->addToLegend();
    _graph_pdo_map[esc_id][index]=graph_pdo;
}
/************************************* GENERATE GRAPH ***************************************/
/************************************* RETRIEVE TREE WIDGET ITEM ***************************************/
QTreeWidgetItem * EcGuiPdo::retrieve_treewid_item(const int &esc_id,
                                                  const std::string &esc_type,
                                                  const std::vector<std::string> &pdo_fields,
                                                  const std::string direction)
{
    if(_esc_pdo_map.count(esc_id)==0){
        std::string esc_id_name=esc_type+"_id_"+std::to_string(esc_id);
        QTreeWidgetItem * topLevelrtn = new QTreeWidgetItem();
        topLevelrtn->setText(0,QString::fromStdString(esc_id_name));
        topLevelrtn->setText(1,"");
        topLevelrtn->setText(2,QString::fromStdString(direction));

        _buffer_pdo_map[esc_id].resize(pdo_fields.size());
        uint16_t k=0;
        for(const auto &pdo_fields_value:pdo_fields){
            QTreeWidgetItem * item = new QTreeWidgetItem();
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable | Qt::ItemIsSelectable);
            item->setCheckState(0,Qt::Unchecked);
            item->setText(0,QString::fromStdString(pdo_fields_value));
            topLevelrtn->addChild(item);
            _buffer_pdo_map[esc_id][k].resize(_buffer_size);
            k++;
        }

        _tree_wid->addTopLevelItem(topLevelrtn);
        _pdo_v[esc_id].resize(pdo_fields.size());
        _esc_pdo_map[esc_id]=topLevelrtn;
    }

    return  _esc_pdo_map[esc_id];
}
/************************************* RETRIEVE TREE WIDGET ITEM ***************************************/

/************************************* FILL DATA ***************************************/
void EcGuiPdo::fill_data(const int &esc_id,
                         QTreeWidgetItem * topLevel,
                         const std::vector<std::string> &pdo_fields,
                         const std::vector<float> &pdo)
{
    try{
        /************************************* DATA ***********************************************/
        int k=0;
        for(const auto &pdo_fields_value:pdo_fields){
    
            _buffer_pdo_map[esc_id][k][_counter_buffer]=pdo[k];

            if(_counter_buffer==_buffer_size-1){
                if(topLevel->isExpanded()){
                    topLevel->child(k)->setText(1,QString::number(pdo[k], 'f', 2));
                }
                
                if(topLevel->child(k)->checkState(0)==Qt::Checked){
                    
                    if(_graph_pdo_map.count(esc_id)==0){
                        _graph_pdo_map[esc_id].resize(pdo_fields.size());
                    }

                    if(!_graph_pdo_map[esc_id][k]){
                        // generate graph
                        std::string esc_pdo_name=topLevel->text(0).toStdString()+ "_" + pdo_fields_value;
                        create_graph(esc_id,k,esc_pdo_name);
                    }
                    // add data to lines
                    _graph_pdo_map[esc_id][k]->addData(_buffer_time,_buffer_pdo_map[esc_id][k],true);    
                    //update plot
                    _update_plot |= true;
                }
            }
            k++;
        }
    }catch (const std::out_of_range &oor) {}
}

void EcGuiPdo::onStopPlotting()
{
    for(int i=0;i<_tree_wid->topLevelItemCount();i++){
        auto topLevel =_tree_wid->topLevelItem(i);
        for(int k=0; k< topLevel->childCount(); k++){
            QTreeWidgetItem * item = topLevel->child(k);
            item->setCheckState(0,Qt::Unchecked);
        }
    }
}


void EcGuiPdo::read()
{
    _ms_receive_time= _receive_timer->elapsed();
    _s_receive_time=(double) _ms_receive_time/1000;
    _buffer_time[_counter_buffer]=_s_receive_time;

    /************************************* READ Rx PDOs  ********************************************/
    read_motor_status();
    read_ft_status();
    read_pow_status();
    read_imu_status();
    read_valve_status();
    read_pump_status();
    /************************************* READ Rx PDOs  ********************************************/

    update_plot();
}

void EcGuiPdo::update_plot()
{
    if(_counter_buffer==_buffer_size-1){
        _counter_buffer=0;
        _time_pdo->setText(QString::number(_s_receive_time, 'f', 2));
        if(_update_plot){
            if(!_first_update){
                _custom_plot->clearGraphs();
                _graph_pdo_map.clear();
                _custom_plot->legend->setVisible(true);
                _first_update=true;
            }
            if(_auto_scroll->isChecked()){
                //make key axis range scroll with the data (at a constant range size of 8):
                _custom_plot->xAxis->setRange(_s_receive_time, 8, Qt::AlignRight);
                _custom_plot->replot();
            }
            
            _update_plot=false;
        }
        else{
            if(_first_update){
                _first_update=false;
            }
        }
    }
    else{
        _counter_buffer++;
    }
}

void EcGuiPdo::read_motor_status()
{
    _client->get_motor_status(_motor_status_map);
    for ( const auto &[esc_id, motor_rx_pdo] : _motor_status_map){
        QTreeWidgetItem *topLevel= retrieve_treewid_item(esc_id,"motor",MotorPdoRx::name,"Rx");
        if(MotorPdoRx::make_vector_from_tuple(motor_rx_pdo,_pdo_v[esc_id])){
            fill_data(esc_id,topLevel,MotorPdoRx::name,_pdo_v[esc_id]);
        }
        
        /************************************* ALIGN POSITION SLIDERS with the motor position ********************************************/
        double motor_pos=std::get<2>(motor_rx_pdo);
        if(_slider_map.motor_sw_map.count(esc_id)>0){
            _slider_map.motor_sw_map[esc_id]->set_spinbox_value(1,motor_pos);
        }
        /************************************* ALIGN POSITION SLIDERS with the motor position ********************************************/
    }  
}

void EcGuiPdo::read_ft_status()
{
    _client->get_ft_status(_ft_status_map);
    for ( const auto &[esc_id, ft_rx_pdo] : _ft_status_map){
        QTreeWidgetItem *topLevel= retrieve_treewid_item(esc_id,"ft",FtPdoRx::name,"Rx");
        if(FtPdoRx::make_vector_from_tuple(ft_rx_pdo,_pdo_v[esc_id])){
            fill_data(esc_id,topLevel,FtPdoRx::name,_pdo_v[esc_id]);
        }
    }
}

void EcGuiPdo::read_pow_status()
{
    _client->get_pow_status(_pow_status_map);
    for ( const auto &[esc_id, pow_rx_pdo] : _pow_status_map){
        if(_counter_buffer==_buffer_size-1){
            _battery_level->display(std::get<0>(pow_rx_pdo));
        }
        QTreeWidgetItem *topLevel= retrieve_treewid_item(esc_id,"pow",PowPdoRx::name,"Rx");
        if(PowPdoRx::make_vector_from_tuple(pow_rx_pdo,_pdo_v[esc_id])){
            fill_data(esc_id,topLevel,PowPdoRx::name,_pdo_v[esc_id]);
        }
    }
}

void EcGuiPdo::read_imu_status()
{
    _client->get_imu_status(_imu_status_map);
    for ( const auto &[esc_id, imu_rx_pdo] : _imu_status_map){
        QTreeWidgetItem *topLevel= retrieve_treewid_item(esc_id,"imu",ImuPdoRx::name,"Rx");
        if(ImuPdoRx::make_vector_from_tuple(imu_rx_pdo,_pdo_v[esc_id])){
            fill_data(esc_id,topLevel,ImuPdoRx::name,_pdo_v[esc_id]);
        }
    }
}
void EcGuiPdo::read_valve_status()
{    
    _client->get_valve_status(_valve_status_map);
    for ( const auto &[esc_id, valve_rx_pdo] : _valve_status_map){
        QTreeWidgetItem *topLevel= retrieve_treewid_item(esc_id,"valve",ValvePdoRx::name,"Rx");
        if(ValvePdoRx::make_vector_from_tuple(valve_rx_pdo,_pdo_v[esc_id])){
            fill_data(esc_id,topLevel,ValvePdoRx::name,_pdo_v[esc_id]);
        }
        /************************************* ALIGN VALVE SLIDERS with the actual encoder pos********************************************/
        double enc_pos=std::get<0>(valve_rx_pdo);
        if(_slider_map.valve_sw_map.count(esc_id)>0){
            _slider_map.valve_sw_map[esc_id]->set_spinbox_value(1,enc_pos);
        }
        /************************************* ALIGN VALVE SLIDERS with the actual pressure********************************************/
    }
}

void EcGuiPdo::read_pump_status()
{
    _client->get_pump_status(_pump_status_map);
    for ( const auto &[esc_id, pump_rx_pdo] : _pump_status_map){
        QTreeWidgetItem *topLevel= retrieve_treewid_item(esc_id,"pump",PumpPdoRx::name,"Rx");
        if(PumpPdoRx::make_vector_from_tuple(pump_rx_pdo,_pdo_v[esc_id])){
            fill_data(esc_id,topLevel,PumpPdoRx::name,_pdo_v[esc_id]);
        }

        /************************************* ALIGN PUMP SLIDERS with the actual pressure********************************************/
        double pressure=std::get<0>(pump_rx_pdo);
        if(_slider_map.pump_sw_map.count(esc_id)>0){
            _slider_map.pump_sw_map[esc_id]->set_spinbox_value(0,pressure);
        }
        /************************************* ALIGN PUMP SLIDERS with the actual pressure********************************************/
    }
}
/********************************************************* READ PDO***********************************************************************************************/

/********************************************************* LOG PDO***********************************************************************************************/
void EcGuiPdo::log()
{
    _ec_logger->log_motor_status(_motor_status_map);
    _ec_logger->log_motor_reference(_motor_ref_map); 

    _ec_logger->log_ft_status(_ft_status_map);
    _ec_logger->log_pow_status(_pow_status_map);
    _ec_logger->log_imu_status(_imu_status_map);

    _ec_logger->log_valve_status(_valve_status_map);
    _ec_logger->log_valve_reference(_valve_ref_map);

    _ec_logger->log_pump_status(_pump_status_map);
    _ec_logger->log_pump_reference(_pump_ref_map);
}
/********************************************************* LOG PDO***********************************************************************************************/

/********************************************************* WRITE PDO***********************************************************************************************/
void EcGuiPdo::sync_write()
{
    _motor_ref_map = _motor_reference_map;
    _valve_ref_map = _valve_reference_map;
    _pump_ref_map  = _pump_reference_map;
}

void EcGuiPdo::init_write_pdo()
{
    _motor_reference_map.clear();
    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
        _motor_reference_map[slave_id]={0,0,0,0,0,0,0,0,0,0,0,0};
    }

    _valve_reference_map.clear();
    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
        _valve_reference_map[slave_id]={0,0,0,0,0,0,0,0,0,0,0,0};
    }

    _pump_reference_map.clear();
    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
        _pump_reference_map[slave_id]={0,0,0,0,0,0,0,0,0};
    }

    sync_write();
}

void EcGuiPdo::starting_write(int time_ms)
{
    _time_ms = time_ms;
    double ts=((double) _time_ms)/1000;
    _ec_gui_slider->set_sliders_info(ts,false);
    _send_timer->restart();

    _motors_selected=check_write_device(_slider_map.motor_sw_map);
    _valves_selected=check_write_device(_slider_map.valve_sw_map);
    _pumps_selected=check_write_device(_slider_map.pump_sw_map);
}

void EcGuiPdo::stopping_write()
{
    double ts=((double) _time_ms)/1000;
    _ec_gui_slider->set_sliders_info(ts,true);
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

bool EcGuiPdo::check_write_device(std::map<int, SliderWidget*> slider_map)
{
    for (auto& [slave_id, slider_wid]: slider_map){
        if(slider_wid->is_slider_checked()){
            return true;
        }
    }
    return false;
}

void EcGuiPdo::write_motor_pdo()
{
    if(!_motors_selected){
        return;
    }

    for (auto& [slave_id, slider_wid]:_slider_map.motor_sw_map){
        float motor_pos=slider_wid->get_spinbox_value(1); // DONE read function!!
        std::get<0>(_motor_reference_map[slave_id])=0x00;
        std::get<1>(_motor_reference_map[slave_id])=motor_pos;
        if(slider_wid->is_slider_checked()){
            std::get<0>(_motor_reference_map[slave_id])=slider_wid->get_spinbox_value(0);
            std::get<1>(_motor_reference_map[slave_id])=slider_wid->compute_wave(1,_s_send_time); 
            std::get<2>(_motor_reference_map[slave_id])=slider_wid->compute_wave(2,_s_send_time); 
            std::get<3>(_motor_reference_map[slave_id])=slider_wid->compute_wave(3,_s_send_time); 
            std::get<4>(_motor_reference_map[slave_id])=slider_wid->compute_wave(4,_s_send_time); 
            std::get<5>(_motor_reference_map[slave_id])=slider_wid->compute_wave(5,_s_send_time); 
            std::get<6>(_motor_reference_map[slave_id])=slider_wid->compute_wave(6,_s_send_time); 
            std::get<7>(_motor_reference_map[slave_id])=slider_wid->compute_wave(7,_s_send_time); 
            std::get<8>(_motor_reference_map[slave_id])=slider_wid->compute_wave(8,_s_send_time); 
            std::get<9>(_motor_reference_map[slave_id])=1;
            std::get<10>(_motor_reference_map[slave_id])=slider_wid->get_spinbox_value(10); 
            std::get<11>(_motor_reference_map[slave_id])=slider_wid->compute_wave(11,_s_send_time);  
        }
    }

    _client->set_motor_reference(_motor_reference_map);
}

void EcGuiPdo::write_valve_pdo()
{
    if(!_valves_selected){
        return;
    }

    for (auto& [slave_id, slider_wid]:_slider_map.valve_sw_map){
        float enc_pos=slider_wid->get_spinbox_value(0); // DONE read function!!
        std::get<1>(_valve_reference_map[slave_id])=enc_pos;
        if(slider_wid->is_slider_checked()){
            std::get<0>(_valve_reference_map[slave_id])=slider_wid->compute_wave(0,_s_send_time);
            std::get<1>(_valve_reference_map[slave_id])=slider_wid->compute_wave(1,_s_send_time); 
            std::get<2>(_valve_reference_map[slave_id])=slider_wid->compute_wave(2,_s_send_time); 
            std::get<3>(_valve_reference_map[slave_id])=slider_wid->compute_wave(3,_s_send_time); 
            std::get<4>(_valve_reference_map[slave_id])=slider_wid->compute_wave(4,_s_send_time); 
            std::get<5>(_valve_reference_map[slave_id])=slider_wid->compute_wave(5,_s_send_time); 
            std::get<6>(_valve_reference_map[slave_id])=slider_wid->compute_wave(6,_s_send_time); 
            std::get<7>(_valve_reference_map[slave_id])=slider_wid->compute_wave(7,_s_send_time); 
            std::get<8>(_valve_reference_map[slave_id])=slider_wid->get_spinbox_value(8);
            std::get<9>(_valve_reference_map[slave_id])=slider_wid->get_spinbox_value(9); 
            std::get<10>(_valve_reference_map[slave_id])=slider_wid->get_spinbox_value(10); 
            std::get<11>(_valve_reference_map[slave_id])=slider_wid->compute_wave(11,_s_send_time);  
        }
    }
    
    _client->set_valve_reference(_valve_reference_map);
}

void EcGuiPdo::write_pump_pdo()
{
    if(!_pumps_selected){
        return;
    }

    for (auto& [slave_id, slider_wid]:_slider_map.pump_sw_map){
        float pressure=slider_wid->get_spinbox_value(0); // DONE read function!!
        std::get<0>(_pump_reference_map[slave_id])=pressure;
        if(slider_wid->is_slider_checked()){
            std::get<0>(_pump_reference_map[slave_id])=slider_wid->compute_wave(0,_s_send_time);
            std::get<1>(_pump_reference_map[slave_id])=slider_wid->get_spinbox_value(1);
            std::get<2>(_pump_reference_map[slave_id])=slider_wid->get_spinbox_value(2); 
            std::get<3>(_pump_reference_map[slave_id])=slider_wid->get_spinbox_value(3);
            std::get<4>(_pump_reference_map[slave_id])=slider_wid->get_spinbox_value(4);
            std::get<5>(_pump_reference_map[slave_id])=slider_wid->get_spinbox_value(5);
            std::get<6>(_pump_reference_map[slave_id])=slider_wid->get_spinbox_value(6);
            std::get<7>(_pump_reference_map[slave_id])=slider_wid->get_spinbox_value(7); 
            std::get<8>(_pump_reference_map[slave_id])=slider_wid->get_spinbox_value(8); 
        }
    }

    _client->set_pump_reference(_pump_reference_map);
}
/********************************************************* WRITE PDO***********************************************************************************************/