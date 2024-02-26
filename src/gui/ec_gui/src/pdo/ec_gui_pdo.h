#ifndef EC_GUI_PDO_H
#define EC_GUI_PDO_H

#include <QtUiTools>
#include <QWidget>

#include "utils/ec_utils.h"
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
      void set_filter(bool first_send, int time_ms);
      void set_ctrl_mode(float ctrl_cmd);
      void write();

private:
      EcIface::Ptr _client;
      EcGuiSlider::Ptr _ec_gui_slider;
      EcGuiSlider::slider_map_t _slider_map;
      
      //************************ READ PDO ******************************
      
      QTreeWidget *_tree_wid;
      QElapsedTimer *_receive_timer;
      QCustomPlot *_custom_plot;
      QPushButton * _stop_plotting_btn;
      std::map<std::string,QCPGraph *> _graph_pdo_map;
      std::map<std::string,QColor> _color_pdo_map;
      bool _update_plot,_first_update,_clear_plot;
      qint64 _ms_receive_time;
      double _s_receive_time;
      float _currentHue = 0.0;
      
      QList<QString> _motor_pdo_fields= {"Link Position",
                                         "Motor Position",
                                         "Link Velocity",
                                         "Motor Velocity",
                                         "Torque",
                                         "Motor Temperature",
                                         "Board Temperature",
                                         "fault",
                                         "rtt",
                                         "op_idx_ack",
                                         "aux",          
                                         "Brake_Sts",
                                         "LED_Sts"};
                                        
       QList<QString> _ft6_pdo_fields=  {"force_x",
                                         "force_y",
                                         "force_z",
                                         "torque_x",
                                         "torque_y",
                                         "torque_z"};
                                         
       QList<QString> _pow_pdo_fields=  {"v_batt",
                                         "v_load",
                                         "i_load",
                                         "temp_pcb",
                                         "temp_heatsink",
                                         "temp_batt"};
                                         
        QList<QString> _imu_pdo_fields=  {"ang_vel_x",
                                          "ang_vel_y",
                                          "ang_vel_z",
                                          "lin_acc_x",
                                          "lin_acc_y",
                                          "lin_acc_z",
                                          "orientation_x",
                                          "orientation_y",
                                          "orientation_z",
                                          "orientation_w"};
                                          
        void create_color(std::string esc_id_pdo);
        QTreeWidgetItem * search_slave_into_treewid(std::string esc_id_name);
        QTreeWidgetItem * initial_setup(std::string esc_id_name,QList<QString> pdo_fields);
        void fill_data(std::string esc_id_name,QTreeWidgetItem * topLevel,QList<QString> pdo_fields,std::vector<float> pdo);
        void onStopPlotting();
        
        void update_plot();
        void read_motor_status();
        void read_ft6_status();
        void read_pow_status();
        void read_imu_status();
        //************************ READ PDO ******************************
        
        //************************ WRITE PDO ******************************
        int _time_ms;
        bool _first_send;
        double filtering(SecondOrderFilter<double>::Ptr filter,double actual_value);
        
        std::vector<MR> _motors_ref;
        RefFlags _motor_ref_flags;
        std::vector<float> _gains;
        float _ctrl_cmd;
        //************************ WRITE PDO ******************************

};

#endif // EC_GUI_PDO_H


