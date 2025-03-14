#ifndef EC_GUI_SLIDER_H
#define EC_GUI_SLIDER_H

#include <QtUiTools>
#include <QWidget>

#include "slider_window.h"
#include "slider_widget.h"
#include "utils/ec_utils.h"
#include <cfloat>

static SliderWidget::slider_info_s motor_info={
  MotorPdoTx::name,
  {"[uless]","[rad]","[rad/s]","[Nm]---[A]","[arbu]", "[arbu]","[arbu]","[arbu]","[arbu]","[uless]","[uless]","[arbu]"},
  {0,2,2,2,4,4,4,4,4,0,0,2},
  {"0","-3.14","-6.28","-10.0","0","0","0","0","0","0","0","-10000"},
  {"500","3.14","6.28","10.0","10000","10000","10000","10000","10000","2","65535","10000"},
  {0,1,1,1,2,2,2,2,2,3,3,1}
};

static const SliderWidget::slider_info_s valve_info= {
  ValvePdoTx::name,
  {"[mA]", "[um]", "[N]","[arbu]", "[arbu]","[arbu]","[arbu]","[arbu]","[uless]","[uless]","[uless]","[arbu]"},
  {2,2,2,4,4,4,4,4,0,0,0,2},
  {"-25.0","-2000.0","-100","-100.0","-100","-100","-100","-100","0","0","0","-10000"},
  {"25.0","2000.0","100","100.0","100","100","100","100","1","1","1","10000"},
  {1,1,1,2,2,2,2,2,3,3,3,1}
};

static const SliderWidget::slider_info_s pump_info= {
  PumpPdoTx::name,
  {"[bar]", "[uless]", "[uless]","[uless]", "[uless]","[uless]","[uless]","[uless]","[uless]"},
  {0,0,0,0,0,0,0,0,0},
  {"0","0","0","0","0","0","0","0","0"},
  {"255","255","255","65535","255","255","255","255","255"},
  {1,1,1,1,1,1,1,1}
};



class EcGuiSlider : public QWidget
{
    Q_OBJECT

public:

    struct slider_map_t{
      std::map<int, SliderWidget*> motor_sw_map;
      std::map<int, SliderWidget*> valve_sw_map;
      std::map<int, SliderWidget*> pump_sw_map;
    };

    struct device_ctrl_t{
      std::map<int32_t,std::map<int32_t,std::vector<float>>> device_gains;
      std::map<int32_t,std::vector<float>> device_limits;
    };

    typedef std::shared_ptr<EcGuiSlider> Ptr;
    
    explicit EcGuiSlider(QWidget *parent = nullptr);

    ~EcGuiSlider();

    slider_map_t get_sliders();
    
    void create_sliders(SSI device_info,device_ctrl_t device_ctrl);
    void delete_sliders();
    void reset_sliders();
    void enable_sliders();
    void disable_sliders();
    void check_sliders();
    void uncheck_sliders();
    void set_sliders_info(double st,bool stopping_wave);
    int get_control_mode(std::string tab_name);
    void enable_control_mode(const std::string& tab_name);
    void disable_control_mode(const std::string& tab_name);

private slots:
    void control_mode_change();

private:
  
  slider_map_t _slider_map;

  float _control_mode;

  QTabWidget *_devicecontrol;
  std::map<std::string,SliderWindow *> _sliders_window_map;
  QVBoxLayout *_sliders_motorlayout;
  QVBoxLayout *_sliders_valvelayout,*_sliders_pumplayout;
  QListWidget* _device_list_wid;
  std::map<int32_t,std::map<int32_t,std::vector<float>>> _device_gains;
  void delete_items(QLayout * layout);
  void on_checkbox_clicked(QCheckBox *,int);
  void set_control_mode(const std::string &tab_name);
  QVBoxLayout* retrieve_slider_layout(const std::string &tab_name,
                                      const QStringList &control_mode,
                                      const std::vector<int> control_mode_hex);
  device_ctrl_t _device_ctrl;
  int _ctrl_mode,_old_ctrl_mode;
};

#endif // EC_GUI_SLIDER_H
