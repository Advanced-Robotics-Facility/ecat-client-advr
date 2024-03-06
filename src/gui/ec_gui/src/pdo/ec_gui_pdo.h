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
    
    QList<QString> _motor_pdo_fields;
    QList<QString> _pow_pdo_fields;
    QList<QString> _ft6_pdo_fields;
    QList<QString> _imu_pdo_fields;                                         
    QList<QString> _valve_pdo_fields,_pump_pdo_fields;
                                        
    void create_color(std::string esc_id_pdo);
    QTreeWidgetItem * search_slave_into_treewid(std::string esc_id_name);
    QList<QString> get_pdo_fields(const std::vector<std::string> pdo_name);
    QTreeWidgetItem * initial_setup(std::string esc_id_name,QList<QString> pdo_fields);
    void fill_data(std::string esc_id_name,QTreeWidgetItem * topLevel,QList<QString> pdo_fields,std::vector<float> pdo);
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
    bool _first_send;
    double filtering(SecondOrderFilter<double>::Ptr filter,double actual_value);
    
    MotorReferenceMap _motors_ref;
    RefFlags _motor_ref_flags;
    std::vector<float> _gains;
    float _ctrl_cmd;
    std::vector<float> _motor_rx_v;
    std::vector<float> _pow_rx_v;
    std::vector<float> _ft_rx_v;
    std::vector<float> _imu_rx_v;
    std::vector<float> _valve_rx_v,_pump_rx_v;
    //************************ WRITE PDO ******************************

};

#endif // EC_GUI_PDO_H


