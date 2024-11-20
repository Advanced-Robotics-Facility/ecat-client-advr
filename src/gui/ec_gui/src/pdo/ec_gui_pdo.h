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
    
      void restart_ec_gui_pdo(EcIface::Ptr client);
      void restart_receive_timer();
      void read();
      void set_filter(int time_ms);
      void restart_send_timer();
      void write();

private:
      EcIface::Ptr _client;
      EcGuiSlider::Ptr _ec_gui_slider;
      EcGuiSlider::slider_map_t _slider_map;
      
      //************************ READ PDO ******************************
      
      QTreeWidget *_tree_wid;
      QElapsedTimer *_receive_timer;
      QElapsedTimer *_send_timer;
      QCustomPlot *_custom_plot;
      QPushButton * _stop_plotting_btn;
      QLabel *_time_pdo;
      std::map<std::string,QCPGraph *> _graph_pdo_map;
      std::map<std::string,QVector<double>> _buffer_pdo_map;
      QVector<double> _buffer_time;
      bool _update_plot,_first_update;
      qint64 _ms_receive_time,_ms_send_time;
      double _s_receive_time,_s_send_time;
      float _currentHue = 0.0;
      uint16_t _counter_buffer;
      uint16_t _buffer_size;

      std::string _esc_id_pdo;
      
      // last received motor data
      MotorStatusMap _motor_status_map;
      // last received ft data
      FtStatusMap _ft_status_map;
      // last received pow data
      PwrStatusMap _pow_status_map;
      // last received imu data
      ImuStatusMap _imu_status_map;
      // last received valve data
      ValveStatusMap _valve_status_map;
      // last received pump data
      PumpStatusMap _pump_status_map;

      std::map<std::string,std::vector<float>>  _pdo_v;

      QLCDNumber *_battery_level;
    
      void create_graph(std::string esc_id_pdo);
      QTreeWidgetItem * search_slave_into_treewid(std::string esc_id_name);
      QTreeWidgetItem * retrieve_treewid_item(const std::string &esc_id_name,
                                              const std::vector<std::string> &pdo_fields,
                                              const std::string &direction);
      void fill_data(const std::string &esc_id_name,
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
      
      MotorReferenceMap _motors_ref;

      ValveReferenceMap _valves_ref;
      
      PumpReferenceMap _pumps_ref;

      void write_motor_pdo();
      void write_valve_pdo();
      void write_pump_pdo();

    //************************ WRITE PDO ******************************

};

#endif // EC_GUI_PDO_H


