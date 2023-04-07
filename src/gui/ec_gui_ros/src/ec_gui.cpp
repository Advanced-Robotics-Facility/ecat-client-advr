#include "ec_gui.h"
#include <fmt/format.h>
#include <QtUiTools>

#include <robot_monitoring/custom_qt_widget_impl.h>
#include "pdo/pdo_widget.h"

EcGui::EcGui(QWidget *parent) :
    QWidget(parent)
{
    // main layout (horizontal)
    auto layout = new QHBoxLayout(this);

    _msg_Txt=new QTextEdit();
    _msg_Txt->setReadOnly(true);

    auto slave_wid = new SlaveWidget();

    XBot::Ui::CustomQtWidget::Args args;
    slave_wid->init(args);

    auto pdo_wid  = new PDOWidget(this);
    auto rvix_wid = new RvizWidget(this);

    auto main_splitter = new QSplitter;
    main_splitter->setOrientation(Qt::Orientation::Horizontal);

    // left vertical layout
    auto lsplitter = new QSplitter;
    lsplitter->setOrientation(Qt::Orientation::Vertical);

    _master_cmd_wid = new MasterCmdWidget(this,_msg_Txt,slave_wid);
    _master_cmd_wid->set_repl_status(REPL_STATUS::OPERATIONAL);

    lsplitter->addWidget(_master_cmd_wid);
    lsplitter->setStretchFactor(0,0);
    lsplitter->addWidget(pdo_wid);
    lsplitter->setStretchFactor(1, 1);

    main_splitter->addWidget(lsplitter);



    // right vertical layout
    auto rsplitter = new QSplitter;
    rsplitter->setOrientation(Qt::Orientation::Vertical);


    auto * tab_wid = new QTabWidget;
    tab_wid->addTab(rvix_wid,"Robot");
    tab_wid->addTab(_msg_Txt,"Message");

    rsplitter->addWidget(tab_wid);
    rsplitter->setStretchFactor(1, 1);
    main_splitter->addWidget(rsplitter);


    auto * main_tab_wid = new QTabWidget;
    main_tab_wid->addTab(main_splitter,"EtherCAT Analyzer");
    main_tab_wid->addTab(slave_wid,"EtherCAT Motors");

    // set layout
    layout->addWidget(main_tab_wid);
    setLayout(layout);
}
