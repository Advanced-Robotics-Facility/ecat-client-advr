#include <stdlib.h>
#include <boost/asio.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "ec_gui_start.h"
#include <QApplication>

#include "utils/ec_utils.h"

void catchUnixSignals(std::initializer_list<int> quitSignals) {
    auto handler = [](int sig) -> void {
        // blocking and not aysnc-signal-safe func are valid
        printf("\nquit the application by signal(%d).\n", sig);
        QCoreApplication::quit();
    };
    
    sigset_t blocking_mask;   
    sigemptyset(&blocking_mask);  
    for (auto sig : quitSignals) 
        sigaddset(&blocking_mask, sig);  
        
    struct sigaction sa;   
    sa.sa_handler = handler;   
    sa.sa_mask    = blocking_mask;  
    sa.sa_flags   = 0;    
    
    for (auto sig : quitSignals)   
        sigaction(sig, &sa, nullptr);
}

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    catchUnixSignals({SIGQUIT, SIGINT, SIGTERM, SIGHUP});
    QApplication a(argc, argv);

    QMainWindow mw;
    EcGuiStart w;
    mw.setCentralWidget(w.centralWidget());
    mw.setWindowState(Qt::WindowMaximized);
    mw.show();
    return a.exec();
}
