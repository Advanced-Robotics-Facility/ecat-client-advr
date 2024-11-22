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
                           const std::vector<int> control_mode_hex,
                           QWidget *parent) :
    QWidget(parent),
    _control_mode_hex(control_mode_hex)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);

    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);

    setLayout(layout);

    if(control_mode.size()!=static_cast<int>(_control_mode_hex.size())){
        throw std::runtime_error("control mode size vectors different size!");
    }

    _control_mode = findChild<QComboBox *>("ControlMode");
    _control_mode->addItems(control_mode);

    _layout= findChild<QVBoxLayout *>("Sliders");

    _actual_control_mode=0x00;
}

QVBoxLayout* SliderWindow::get_layout()
{
    return _layout;
}


int SliderWindow::read_control_mode()
{
    int control_mode_index=_control_mode->currentIndex();
    _actual_control_mode=_control_mode_hex[control_mode_index];
    return _actual_control_mode;
}

QComboBox* SliderWindow::get_control_mode()
{
    return _control_mode;
}

void SliderWindow::enable_control_mode()
{
    _control_mode->setEnabled(true);
}

void SliderWindow::disable_control_mode()
{
    _control_mode->setEnabled(false);
}

SliderWindow::~SliderWindow()
{}