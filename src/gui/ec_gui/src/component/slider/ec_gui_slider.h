#ifndef EC_GUI_SLIDER_H
#define EC_GUI_SLIDER_H

#include <QtUiTools>
#include <QWidget>

#include "slider_window.h"
#include "slider_widget.h"
#include "utils/ec_utils.h"
#include <cfloat>

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
  void adjust_slave_info(SliderWidget::slider_info_s &slide_info,
                         int index,
                         std::vector<std::string> new_fields);
};

#endif // EC_GUI_SLIDER_H
