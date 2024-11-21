#include "slider_window.h"
#include <iostream>

inline void initSliderWindowResource()
{
     Q_INIT_RESOURCE(ec_gui_start_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    
    initSliderWindowResource();
    
    QUiLoader loader;

    QFile file(":/component/slider/slider_window.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
    
    
}
}

SliderWindow::SliderWindow(const QStringList&  control_mode,
                           QWidget *parent) :
    QWidget(parent)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);

    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);

    setLayout(layout);

    _control_mode = findChild<QComboBox *>("ControlMode");
    _control_mode->addItems(control_mode);

    _layout= findChild<QVBoxLayout *>("Sliders");
}
QVBoxLayout* SliderWindow::get_layout()
{
    return _layout;
}

SliderWindow::~SliderWindow()
{}