#ifndef EC_GUI_SLIDER_H
#define EC_GUI_SLIDER_H

#include <QtUiTools>
#include <QWidget>

#include "slider_widget.h"
#include "utils/ec_utils.h"

static const SliderWidget::slider_info_s motor_info={
  MotorPdoTx::name,
  {"[uless]","[rad]","[rad/s]","[Nm]","[arbu]", "[arbu]","[arbu]","[arbu]","[arbu]","[uless]","[uless]","[arbu]"},
  {0,2,2,2,4,4,4,4,4,0,0,2},
  {"0","-3.14","-6.28","-10.0","0","0","0","0","0","0","0","-10000"},
  {"500","3.14","6.28","10.0","10000","10000","10000","10000","10000","2","65535","10000"},
  {0,1,1,1,2,2,2,2,2,0,0,1}
};

static const SliderWidget::slider_info_s valve_info= {
  ValvePdoTx::name,
  {"[mA]", "[uless]", "[uless]","[uless]", "[uless]","[s]","[uless]","[arbu]"},
  {2,0,0,0,0,0,0,2},
  {"-25.0","0","0","0","0","0","0","0.0"},
  {"25.0","100.0","100.0","5.0","100","100","10","10000"},
  {1,1,1,1,1,1,1,1}
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

    typedef std::shared_ptr<EcGuiSlider> Ptr;
    
    explicit EcGuiSlider(QWidget *parent = nullptr);

    ~EcGuiSlider();

    slider_map_t get_sliders();
    
    void create_sliders(SSI device_info);
    void delete_sliders();
    void reset_sliders();
    void enable_sliders();
    void disable_sliders();
    void set_sliders_filter(double st);

private:
  
  slider_map_t _slider_map;

  float _control_mode;

  QVBoxLayout *_sliders_motorlayout;
  QVBoxLayout *_sliders_valvelayout,*_sliders_pumplayout;
  QListWidget* _device_list_wid;
  void delete_items(QLayout * layout);
  void on_checkbox_clicked(QCheckBox *,int);
};

#endif // EC_GUI_SLIDER_H
