#include <stdlib.h>
#include <boost/asio.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "ec_gui_start.h"
#include <QApplication>

#include "utils/ec_utils.h"

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QApplication a(argc, argv);

    QMainWindow mw;
    EcGuiStart w;
    mw.setCentralWidget(w.centralWidget());
    mw.setWindowState(Qt::WindowMaximized);
    mw.show();
    return a.exec();
}
