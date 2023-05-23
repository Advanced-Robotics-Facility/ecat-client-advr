#ifndef EC_GUI_SLIDER_H
#define EC_GUI_SLIDER_H

#include <QtUiTools>
#include <QWidget>

#include "slider_widget.h"

class EcGuiSlider : public QWidget
{
    Q_OBJECT

public:

    struct joint_info_t{

    std::string joint_name;

    double min_pos;
    double max_pos;
    double max_vel;
    double max_torq;

    double actual_pos;
    double actual_vel;
    double actual_torq;

    };
    
    struct slider_map_t{
        
    std::map<int, SliderWidget*> position_sw_map;
    std::map<int, SliderWidget*> velocity_sw_map;
    std::map<int, SliderWidget*> position_t_sw_map;
    std::map<int, SliderWidget*> torque_sw_map;
    std::map<int, SliderWidget*> actual_sw_map_selected;
    
    };

    typedef std::shared_ptr<EcGuiSlider> Ptr;
    
    explicit EcGuiSlider(std::map<int ,joint_info_t > joint_info_map,
                        QWidget *parent = nullptr);

    ~EcGuiSlider();

    slider_map_t get_sliders();
    void set_actual_sliders(std::map<int, SliderWidget*> actual_sw_map_selected);
    void reset_sliders();
    void enable_sliders();
    void disable_sliders();

private:
  
  std::map<int ,joint_info_t > _joint_info_map;
  slider_map_t _slider_map;

  float _control_mode;

  QVBoxLayout *_sliders_poslayout,*_sliders_vellayout,*_sliders_torqlayout;

};

#endif // EC_GUI_SLIDER_H
