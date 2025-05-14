#include "ec_gui_net.h"
#include "iostream"

#include <QNetworkInterface>
#include <QHostAddress>

#define USERNAME_COL 1
#define HOSTNAME_COL 2
#define HOSTPORT_COL 3
#define TERMINAL_COL 4

EcGuiNet::EcGuiNet(QWidget *parent) :
    QWidget(parent)
{
    _net_tree_wid = parent->findChild<QTreeWidget *>("NetworkSetup");
    _net_tree_wid->installEventFilter(this);
    
    _net_item = nullptr;
    _net_column=-1;
    
    _server_username=_net_tree_wid->topLevelItem(0)->child(1)->text(1);
    QString client_user_name = qgetenv("USER");
    _net_tree_wid->topLevelItem(0)->child(0)->setText(USERNAME_COL,client_user_name);
    
    if(_net_tree_wid->topLevelItem(0)->child(1)->text(2)=="localhost") {
        _server_hostname="127.0.0.1";
    }
    else{
        _server_hostname=_net_tree_wid->topLevelItem(0)->child(1)->text(2);
    }
    _server_port=_net_tree_wid->topLevelItem(0)->child(1)->text(3);
    
    _password = parent->findChild<QLineEdit *>("Password");
    connect(_password, &QLineEdit::textChanged, this, &EcGuiNet::OnPasswordChanged);
    connect(_password, &QLineEdit::returnPressed, this, &EcGuiNet::OnPasswordEntered);
    
    _server_pwd= _password->text();
    
    connect(_net_tree_wid, SIGNAL(itemDoubleClicked(QTreeWidgetItem*, int)),this, SLOT(OnMouseClicked(QTreeWidgetItem*, int)));

    _gui_file_path= QDir::tempPath()+"/ec_gui_log.txt";
    _ec_master_file_path =QDir::tempPath()+"/ec_master_log.txt";
    _ec_master_process = new QProcess(this);
    _ec_master_process->setReadChannel(QProcess::StandardOutput);
    _ec_master_process->setProcessChannelMode(QProcess::MergedChannels);
    _ec_master_process->setCurrentReadChannel(QProcess::StandardOutput);

    connect(_ec_master_process , SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(ec_master_processFinished(int, QProcess::ExitStatus)));
    connect(_ec_master_process, &QProcess::readyReadStandardOutput,this, &EcGuiNet::ec_master_readyStdO);

    _server_file_path =QDir::tempPath()+"/udp_server_log.txt";
    _server_process = new QProcess(this);
    _server_process->setReadChannel(QProcess::StandardOutput);
    _server_process->setProcessChannelMode(QProcess::MergedChannels);
    _server_process->setCurrentReadChannel(QProcess::StandardOutput);

    connect(_server_process , SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(server_processFinished(int, QProcess::ExitStatus)));
    connect(_server_process, &QProcess::readyReadStandardOutput,this, &EcGuiNet::server_readyStdO);
    
    /*protocl */
    _protocol_combobox = parent->findChild<QComboBox *>("Protocol");
    _protocol_combobox->setCurrentIndex(0);
    _server_protocol = _protocol_combobox->currentText();
    
    /* connection of frequency function */
    connect(_protocol_combobox, SIGNAL(currentIndexChanged(int)),this,SLOT(OnProtocolChanged()));
    set_ec_network();
}

void EcGuiNet::OnPasswordEntered()
{
    _password->setEchoMode(QLineEdit::Password);
}

void EcGuiNet::OnPasswordChanged()
{
    _password->setEchoMode(QLineEdit::PasswordEchoOnEdit);
    _server_pwd= _password->text();
}

void EcGuiNet::set_protocol_enabled(bool enable)
{
    _protocol_combobox->setEnabled(enable);
}

void EcGuiNet::OnProtocolChanged()
{
    _server_protocol = _protocol_combobox->currentText();
    set_ec_network();
}

void EcGuiNet::set_ec_network()
{
    QString client_host_name;
    for (const QNetworkInterface &netInterface: QNetworkInterface::allInterfaces()) {
        if(netInterface.type()==QNetworkInterface::Ethernet ||
           netInterface.type()==QNetworkInterface::Wifi){
            bool ip_found=false;
            for (const QNetworkAddressEntry &address: netInterface.addressEntries()) {
                if(address.ip().protocol() == QAbstractSocket::IPv4Protocol){
                    ip_found=true;
                    client_host_name=address.ip().toString();
                }
            }
            if(netInterface.type()==QNetworkInterface::Ethernet && ip_found){
                break;
            } 
        }
    }

    auto client_user_name=_net_tree_wid->topLevelItem(0)->child(0)->text(1);
    _server_username=_net_tree_wid->topLevelItem(0)->child(1)->text(1);
    if(_server_username!=client_user_name){
        _real_server_username=_server_username;
    }
    _server_username=_real_server_username;
    
    if(_net_tree_wid->topLevelItem(0)->child(1)->text(2)=="localhost"){
        _server_hostname="127.0.0.1";
        client_host_name="localhost";
        _server_username=client_user_name;
    }
    else{
        _server_hostname=_net_tree_wid->topLevelItem(0)->child(1)->text(2);
        if(_server_hostname=="127.0.0.1"){
            client_host_name="127.0.0.1";
            _server_username=client_user_name;
        }
    }
    _net_tree_wid->topLevelItem(0)->child(0)->setText(HOSTNAME_COL,client_host_name);

    _net_tree_wid->topLevelItem(0)->child(1)->setText(USERNAME_COL,_server_username);

    _net_tree_wid->topLevelItem(0)->child(2)->setText(USERNAME_COL,_server_username);
    _net_tree_wid->topLevelItem(0)->child(2)->setText(HOSTNAME_COL,_net_tree_wid->topLevelItem(0)->child(1)->text(2));
    
    _server_port=_net_tree_wid->topLevelItem(0)->child(1)->text(3);
    QString client_port=_server_port;
    if(_server_protocol=="udp"){
        client_port=QString::number(_server_port.toInt()-1);
    }
    _net_tree_wid->topLevelItem(0)->child(0)->setText(HOSTPORT_COL,client_port);
    
    _net_tree_wid->closePersistentEditor(_net_item,_net_column); // close old editor
    _net_item = nullptr;
    _net_column=-1;
    _net_enabled=true;
}

bool EcGuiNet::eventFilter( QObject* o, QEvent* e )
{
    if( o == _net_tree_wid && e->type() == QEvent::KeyRelease){
        QKeyEvent *qkey = static_cast<QKeyEvent*>(e);
        if(qkey->key() == Qt::Key_Return){
            set_ec_network();
            //return true;
        }
    }
    return false;
}

void EcGuiNet::set_net_enabled(bool enable)
{
    _net_enabled=enable;
    if(!_net_enabled){
        close_net_setup();
    }
}

void EcGuiNet::close_net_setup()
{
    if(_net_item != nullptr) {
        _net_tree_wid->closePersistentEditor(_net_item,_net_column); // close old editor
    }
}

void EcGuiNet::OnMouseClicked(QTreeWidgetItem* item, int column)
{
    if(_net_item != nullptr){
        _net_tree_wid->closePersistentEditor(_net_item,_net_column); // close old editor
    }

    _net_item = item;
    _net_column=column;
    
    if(_net_enabled){
        if((item->text(0)=="Server") &&
           ((column == USERNAME_COL) || 
            (column == HOSTNAME_COL)   ||
            (column == HOSTPORT_COL))){
            _net_tree_wid->openPersistentEditor(item,column);
        }
        else{
            _net_tree_wid->closePersistentEditor(item,column);
        }
    }

    if(column==TERMINAL_COL){
        bool error_on_terminal=true;
        if(item->text(0)=="Server"){
            if(_server_file){
                error_on_terminal=false;
                view_server_process();
            }
        }
        else if(item->text(0)=="EtherCAT Master"){
            if(_ec_master_file){
                error_on_terminal=false;
                view_master_process();
            }
        }
        else{
            if(QFile(_gui_file_path).exists()){
                error_on_terminal=false;
                view_gui_process();
            }
        }
        
        if(error_on_terminal){
            QMessageBox msgBox;
            msgBox.critical(this,msgBox.windowTitle(),
            tr("Cannot open the terminal requested!, log file not found!\n"));
        }
    }
}

void EcGuiNet::ec_master_processFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    ec_master_readyStdO();
    if(_open_config_file){
        save_config_file();
    }
}

void EcGuiNet::server_processFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    server_readyStdO();
}

void EcGuiNet::kill_view_process(QString& terminal_pid)
{
    if(terminal_pid!=""){
        QStringList cmd={"-9"};
        cmd.append(terminal_pid);
        QProcess kill_view_proc;
        kill_view_proc.start("kill",cmd);
        kill_view_proc.waitForFinished();
    }
    terminal_pid="";
}

void EcGuiNet::view_process(const QString &file_path,QString &terminal_pid)
{
    kill_view_process(terminal_pid);

    QStringList cmd={"-x","echo","$$",">","/tmp/terminal_pid.txt"};
    cmd.append("&&");
    cmd.append({"tail","-f","-n","+1"});
    cmd.append(file_path);

    QProcess view_proc;
    view_proc.start("terminator",cmd);
    if(view_proc.waitForFinished()){
        QFile file("/tmp/terminal_pid.txt");
        if (file.open(QFile::ReadOnly)){
            QTextStream in(&file);
            while (!in.atEnd()){
                terminal_pid= in.readLine();
                break;
            }
        }
        file.close();
        file.remove();
    }
}

void EcGuiNet::view_gui_process()
{
    view_process(_gui_file_path,_gui_terminal_pid);
}

void EcGuiNet::view_master_process()
{
    view_process(_ec_master_file_path,_master_terminal_pid);
}

void EcGuiNet::ec_master_readyStdO()
{
    if(_ec_master_process->canReadLine()){
        _ec_master_stdout = _ec_master_process->readAllStandardOutput();
        if(_ec_master_file && _ec_master_stream){
            if(_ec_master_file->isOpen()){
                *_ec_master_stream << _ec_master_stdout;
                _ec_master_stream->flush();
            }
        } 
    }
}

void EcGuiNet::view_server_process()
{
    view_process(_server_file_path,_server_terminal_pid);
}

void EcGuiNet::server_readyStdO()
{
    if(_server_process->canReadLine()){
        _server_stdout = _server_process->readAllStandardOutput();
        if(_server_file && _server_stream){
            if(_server_file->isOpen()){
                *_server_stream << _server_stdout;
                _server_stream->flush();
            }
        } 
    }
}

QString EcGuiNet::find_running_process(QProcess * process,QString bin_name,QString& stdout)
{
    QStringList cmd;
    QString bin_pid="";
    
    cmd = _ssh_command;
    cmd.append("'pgrep'"); // remember comment out: .bashrc all line of # If not running interactively, don't do anything
    cmd.append(bin_name);
    
    stdout.clear();
    process->start("sshpass", cmd);
    if(process->waitForFinished()){
       bin_pid = stdout;
    }
    bin_pid = bin_pid.remove(QChar('\n'));

    return bin_pid;
}


QString EcGuiNet::find_process(QProcess * process,QString bin_name,QString& stdout)
{
    QStringList cmd;
    QString bin_file_path="";
    
    cmd = _ssh_command;
    cmd.append("'which'"); // remember comment out: .bashrc all line of # If not running interactively, don't do anything
    cmd.append(bin_name);
    
    stdout.clear();
    process->start("sshpass", cmd);
    if(process->waitForFinished()){
       bin_file_path = stdout;
    }
    
    bin_file_path = bin_file_path.remove(QChar('\n'));

    return bin_file_path;
}

void EcGuiNet::kill_process(QProcess *process,QString bin_name,QString& stdout)
{
    QString pid=find_running_process(process,bin_name,stdout);
    if(pid!=""){
        QStringList cmd;
        
        cmd=_ssh_command;
        cmd.append("'killall'");
        cmd.append("-9");
        cmd.append(bin_name);
        
        process->start("sshpass", cmd);
        process->waitForFinished();
    }
}

void EcGuiNet::start_process(QProcess *process,QString bin_file_path,QString option)
{
    QStringList cmd;
    cmd.append(_ssh_command); 
    /*Force pseudo-terminal allocation.  This can be used to
    execute arbitrary screen-based programs on a remote
    machine, which can be very useful, e.g. when implementing
    menu services.  Multiple -t options force tty allocation,
    even if ssh has no local tty.*/
    cmd.insert(3,"-tt"); 
    cmd.append(bin_file_path);
    if(option!=""){
        cmd.append(option);
    }
    process->start("sshpass", cmd);
}

bool EcGuiNet::create_ssh_cmd(QProcess *process,QString& stdout)
{
    process->close();

    _ssh_command.clear();
    _ssh_command.append("-p");

    _ssh_command.append(_server_pwd);
    _ssh_command.append("ssh");
    _ssh_command.append("-o StrictHostKeyChecking=no");
    _ssh_command.append(_server_username+"@"+_server_hostname);

    QStringList cmd = _ssh_command;
    cmd.append("'whoami'");
    process->start("sshpass", cmd);
    process->waitForFinished();

    auto user_name = stdout.remove(QChar('\n'));
    
    if(user_name != _server_username){
        QMessageBox msgBox;
        msgBox.setText("Problem on the ssh command, please verify the EtherCAT system setup");
        msgBox.exec();
        return false;
    }

    return true;
}

bool EcGuiNet::start_master_process(const QString &bin_file_name,
                                    const QString &option,
                                    QString &error)
{   
    error=""; 
    kill_process(_ec_master_process,bin_file_name,_ec_master_stdout);
    auto bin_file_path = find_process(_ec_master_process,bin_file_name,_ec_master_stdout);
    
    if(bin_file_path.isEmpty()){
        error="cannot find the " + bin_file_name + " process on machine requested!!";
        return false;
    }

    try{
        start_process(_ec_master_process,bin_file_path,option);
        QFile(_ec_master_file_path).remove();
        _ec_master_file=new QFile(_ec_master_file_path);
        if (_ec_master_file->open(QFile::WriteOnly | QFile::Truncate)) {
            _ec_master_stream=new QTextStream(_ec_master_file);
            view_master_process();
        }
    } catch ( std::exception &e ) {
        error="error on starting " + bin_file_name + " process, " + QString(e.what());
        return false;
    }

    return true;
}

bool EcGuiNet::start_network()
{
    if(!create_ssh_cmd(_ec_master_process,_ec_master_stdout)){
        return false;
    }
    
    /******************************START EtherCAT Master ************************************************/
    QString bin_file_name = "'repl'";
    QString option="";
    if(_server_protocol=="udp"){
        option="-f ~/.ecat_master/configs/zipc_config.yaml";  
    }
    QString cmd_error="";
    if(start_master_process(bin_file_name,option,cmd_error)){
        /******************************START SEVER ************************************************/
        if(_server_protocol=="udp") {
            bin_file_name = "'udp_server'";
            kill_process(_server_process,bin_file_name,_server_stdout);
        
            auto bin_file_path = find_process(_server_process,bin_file_name,_server_stdout);
            if(bin_file_path.isEmpty()){
                cmd_error="cannot find the " + bin_file_name + " process on machine requested!!";
            }
            else{
                try{
                    start_process(_server_process,bin_file_path,"");
                    QFile(_server_file_path).remove();
                    _server_file=new QFile(_server_file_path);
                    if (_server_file->open(QFile::WriteOnly | QFile::Truncate)) {
                        _server_stream=new QTextStream(_server_file);
                        view_server_process();
                    }
                } catch ( std::exception &e ) {
                    cmd_error="error on starting udp server process, " + QString(e.what());
                }
            }
        }
    }

    if(cmd_error!=""){
        stop_network();
        QMessageBox msgBox;
        QString message="Problem on start EtherCAT system command, reason: "+cmd_error;
        msgBox.setText(message);
        msgBox.exec();
        return false;
    }

    return true;
}

bool EcGuiNet::stop_network()
{
    if(!create_ssh_cmd(_ec_master_process,_ec_master_stdout)){
        return false;
    }
    stopping_network(true);
    return true;
}

void EcGuiNet::stopping_network(bool force_stop)
{
    /******************************STOP Server ************************************************/
    if(_server_file){
        if(_server_file->isOpen()){
            _server_file->close();
        }
        _server_file=nullptr;
    }
    if(_server_process->state()!=QProcess::NotRunning || force_stop){
        _server_process->close();
        kill_process(_server_process,"'udp_server'",_server_stdout);
    }
    kill_view_process(_server_terminal_pid);
    /******************************STOP EtherCAT Master ************************************************/
    if(_ec_master_file){
        if(_ec_master_file->isOpen()){
            _ec_master_file->close();
        }
        _ec_master_file=nullptr;
    }
    if(_ec_master_process->state()!=QProcess::NotRunning || force_stop){
        _ec_master_process->close();
        kill_process(_ec_master_process,"'repl'",_ec_master_stdout);
        kill_process(_ec_master_process,"'fw_update'",_ec_master_stdout);
    }
    kill_view_process(_master_terminal_pid);
}

EcGuiNet::ec_net_info_t EcGuiNet::get_net_setup()
{
    ec_net_info_t ec_net_info;

    ec_net_info.protocol=_server_protocol.toStdString();
    ec_net_info.host_name=_server_hostname.toStdString();
    ec_net_info.host_port=_server_port.toUInt();
    
    return ec_net_info;
}

void EcGuiNet::copy_files_network(const QStringList &files_list)
{
    bool show_message=true;
    QMessageBox msgBox;
    msgBox.setText("Problem on copy file(s) command!");
    close_net_setup();
    if(_ec_master_process->state()==QProcess::NotRunning){
        if(!files_list.empty()){
            if(create_ssh_cmd(_ec_master_process,_ec_master_stdout)){
                QStringList scp_cmd={"-p",_server_pwd,"scp"};
                scp_cmd.append("-o StrictHostKeyChecking=no");
                for (const auto &file_path:files_list){
                    scp_cmd.append(file_path);
                }
                scp_cmd.append(_server_username+"@"+_server_hostname+":/$HOME/.ecat_master/firmware/");
                _ec_master_process->start("sshpass", scp_cmd);
                if(_ec_master_process->waitForFinished()){
                    msgBox.setText("File(s) copied!");
                }
            }
            else{
                show_message=false;
            }
        }
        else{
            msgBox.setText("No file(s) selected!");
        }
    }
    if(show_message){
        msgBox.exec();
    }
}

bool EcGuiNet::copy_config_file()
{
    QStringList scp_cmd={"-p",_server_pwd,"scp"};
    scp_cmd.append("-o StrictHostKeyChecking=no");
    scp_cmd.append(_server_username+"@"+_server_hostname+":/$HOME/.ecat_master/configs/microCTRL_config.yaml");
    scp_cmd.append("/tmp/microCTRL_config.yaml");
    _ec_master_process->start("sshpass", scp_cmd);
    return _ec_master_process->waitForFinished();
}

void EcGuiNet::save_config_file()
{
    _open_config_file=false;
    QMessageBox msgBox;
    msgBox.setText("Problem on save configuration file command");
    QStringList scp_cmd={"-p",_server_pwd,"scp"};
    scp_cmd.append("-o StrictHostKeyChecking=no");
    scp_cmd.append("/tmp/microCTRL_config.yaml");
    scp_cmd.append(_server_username+"@"+_server_hostname+":/$HOME/.ecat_master/configs/");
    _ec_master_process->start("sshpass", scp_cmd);
    if(_ec_master_process->waitForFinished()){
        msgBox.setText("Firmware configuration file saved!");
    }
    QFile("/tmp/microCTRL_config.yaml").remove();
    msgBox.exec();
}

void EcGuiNet::open_firmware_config()
{
    bool show_message=true;
    QMessageBox msgBox;
    msgBox.setText("Problem on open configuration file command");
    close_net_setup();
    if(_ec_master_process->state()==QProcess::NotRunning){
        show_message=false;
        if(create_ssh_cmd(_ec_master_process,_ec_master_stdout)){
            if(copy_config_file()){
                QStringList cmd;
                cmd.append("gedit");
                cmd.append("/tmp/microCTRL_config.yaml");
                _ec_master_process->start("sshpass", cmd);
                _open_config_file=true;
            }
        }
    }
    if(show_message){
        msgBox.exec();
    }
}

void EcGuiNet::start_firmware_update()
{
    bool show_message=true;
    QString cmd_error="";
    close_net_setup();
    if(_ec_master_process->state()==QProcess::NotRunning){
        show_message=false;
        if(create_ssh_cmd(_ec_master_process,_ec_master_stdout)){
            kill_process(_ec_master_process,"'repl'",_ec_master_stdout); // kill repl before the fw_update
            if(!start_master_process("'fw_update'","",cmd_error)){
                show_message=true;
            }
        }
    }
    if(show_message){
        stop_firmware_update();
        QMessageBox msgBox;
        QString message="Problem on start firmware update command! " + cmd_error;
        msgBox.setText(message);
        msgBox.exec();
    }
}

void EcGuiNet::stop_firmware_update()
{
    _ec_master_process->close();
    kill_process(_ec_master_process,"'fw_update'",_ec_master_stdout);
    kill_view_process(_master_terminal_pid);
}

EcGuiNet::~EcGuiNet()
{
    stopping_network();
    kill_view_process(_gui_terminal_pid);
}
