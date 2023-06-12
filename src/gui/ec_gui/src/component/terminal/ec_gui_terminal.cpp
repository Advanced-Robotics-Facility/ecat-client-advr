#include "ec_gui_terminal.h"

inline void initSlidersResource()
{
     Q_INIT_RESOURCE(ec_gui_start_resources);
}

namespace
{

QWidget * LoadUiFile(QWidget * parent)
{
    
    initSlidersResource();
    
    QUiLoader loader;

    QFile file(":/component/terminal/ec_gui_terminal.ui");
    file.open(QFile::ReadOnly);

    QWidget *formWidget = loader.load(&file, parent);
    file.close();

    return formWidget;
    
    
}
}

EcGuiTerminal::EcGuiTerminal (QWidget *parent) :
QWidget(parent)
{
    /* Create GUI layout */
    auto * ui = ::LoadUiFile(this);
    
    auto * layout = new QVBoxLayout;
    layout->addWidget(ui);

    setLayout(layout);

    _terminal_txt= findChild<QTextBrowser *>("Terminal");
}

void EcGuiTerminal::setText(QString terminal_line)
{
    _terminal_txt->insertPlainText(terminal_line);
}


EcGuiTerminal::~EcGuiTerminal()
{
    
}
