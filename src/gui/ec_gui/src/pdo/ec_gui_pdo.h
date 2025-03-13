#ifndef EC_GUI_PDO_H
#define EC_GUI_PDO_H

#include <QtUiTools>
#include <QWidget>

#include "ec_gui_slider.h"
#include "qcustomplot.h"


class EcGuiPdo : public QWidget
{
     Q_OBJECT
     
public:
    
      typedef std::shared_ptr<EcGuiPdo> Ptr;
    
      EcGuiPdo(EcGuiSlider::Ptr ec_gui_slider,
               QWidget * parent = 0);
      
      ~EcGuiPdo();
    
      void restart_ec_gui_pdo(EcIface::Ptr client,EcLogger::Ptr ec_logger);
      void restart_receive_timer();
      void read();
      void starting_write(int time_ms);
      void stopping_write();
      void write();
      void sync_write();
      void log();

private:
      EcIface::Ptr _client;
      EcLogger::Ptr _ec_logger;
      EcGuiSlider::Ptr _ec_gui_slider;
      EcGuiSlider::slider_map_t _slider_map;
      
      //************************ READ PDO ******************************
      
      QTreeWidget *_tree_wid;
      QElapsedTimer *_receive_timer;
      QElapsedTimer *_send_timer;
      QCustomPlot *_custom_plot;
      QPushButton * _stop_plotting_btn;
      QCheckBox *_auto_scroll;
      QLabel *_time_pdo;
      std::map<int,QTreeWidgetItem *> _esc_pdo_map;
      std::map<int,std::vector<QCPGraph *>> _graph_pdo_map;
      std::map<int,std::vector<float>>  _pdo_v;
      std::map<int,std::vector<QVector<double>>> _buffer_pdo_map;
      QVector<double> _buffer_time;
      bool _update_plot,_first_update;
      qint64 _ms_receive_time,_ms_send_time;
      double _s_receive_time,_s_send_time;
      float _currentHue = 0.0;
      uint16_t _counter_buffer;
      uint16_t _buffer_size;

      // last received motor data
      MotorStatusMap _motor_status_map;
      MotorReferenceMap _motor_reference_map,_motor_ref_map;
      // last received ft data
      FtStatusMap _ft_status_map;
      // last received pow data
      PwrStatusMap _pow_status_map;
      // last received imu data
      ImuStatusMap _imu_status_map;
      // last received valve data
      ValveStatusMap _valve_status_map;
      ValveReferenceMap _valve_reference_map,_valve_ref_map;
      // last received pump data
      PumpStatusMap _pump_status_map;
      PumpReferenceMap _pump_reference_map,_pump_ref_map;

      QLCDNumber *_battery_level;
    
      void create_graph(const int &esc_id,const int &index,const std::string &esc_pdo_name);
      QTreeWidgetItem * retrieve_treewid_item(const int &esc_id,
                                              const std::string &esc_type,
                                              const std::vector<std::string> &pdo_fields,
                                              const std::string direction);
      void fill_data(const int &esc_id,
                     QTreeWidgetItem * topLevel,
                     const std::vector<std::string> &pdo_fields,
                     const std::vector<float> &pdo);
      void onStopPlotting();

      void update_plot();
      
      void read_motor_status();
      void read_ft_status();
      void read_pow_status();
      void read_imu_status();
      void read_valve_status();
      void read_pump_status();
    //************************ READ PDO ******************************
    
    //************************ WRITE PDO ******************************
      int _time_ms;
      bool check_write_device(std::map<int, SliderWidget*> slider_map);
      void init_write_pdo();
      void write_motor_pdo();
      void write_valve_pdo();
      void write_pump_pdo();

    //************************ WRITE PDO ******************************

    //************************ LOG PDO ******************************
    bool _motors_selected,_valves_selected,_pumps_selected;

};

#endif // EC_GUI_PDO_H


