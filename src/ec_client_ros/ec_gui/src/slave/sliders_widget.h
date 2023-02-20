#ifndef SLIDERSWIDGET_H
#define SLIDERSWIDGET_H


#include <QtUiTools/QtUiTools>
#include <QWidget>
#include <QLabel>
#include <functional>
#include "slider_widget.h"

#include "ec_msgs/SlaveDescription.h"

class SlidersWidget : public QWidget
{
Q_OBJECT
public:

    explicit SlidersWidget (std::vector<std::string>  joint_names,
                            std::vector<ec_msgs::SlaveDescription> slaves_descr_vector,
                            std::string slider_name,
                            QWidget * parent = 0);

    std::map<std::string, SliderWidget *> get_slider_map();

    ~SlidersWidget();

private:

    QLabel *_jname;
    std::vector<std::string>  _joint_names;
    QVBoxLayout *_sliders_leftlayout,*_sliders_rightlayout;
    std::map<std::string, SliderWidget *> _slider_wid_map;

    QTreeWidget *_qwid;
};

#endif // SLIDERSWIDGET_H
