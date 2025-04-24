#include "ec_gui_net.h"
#include "iostream"

#define HOSTNAME_COL 1
#define HOSTIP_COL 2
#define HOSTPORT_COL 3
#define TERMINAL_COL 4

EcGuiNet::EcGuiNet(QWidget *parent) :
    QWidget(parent)
{
    _net_tree_wid = parent->findChild<QTreeWidget *>("NetworkSetup");
    _net_tree_wid->installEventFilter(this);
    
    _net_item = nullptr;
    _net_column=-1;
    
    _server_hostname=_net_tree_wid->topLevelItem(0)->child(1)->text(1);
    
    
    if(_net_tree_wid->topLevelItem(0)->child(1)->text(2)=="localhost") {
        _server_ip="127.0.0.1";
    }
    else{
        _server_ip=_net_tree_wid->topLevelItem(0)->child(1)->text(2);
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


void EcGuiNet::OnProtocolChanged()
{
    _server_protocol = _protocol_combobox->currentText();
    set_ec_network();
}

void EcGuiNet::set_ec_network()
{
    _server_hostname=_net_tree_wid->topLevelItem(0)->child(1)->text(1);
    _net_tree_wid->topLevelItem(0)->child(2)->setText(HOSTNAME_COL,_server_hostname);
    
    _net_tree_wid->topLevelItem(0)->child(2)->setText(HOSTIP_COL,_net_tree_wid->topLevelItem(0)->child(1)->text(2));
    if(_net_tree_wid->topLevelItem(0)->child(1)->text(2)=="localhost"){
        _server_ip="127.0.0.1";
    }
    else{
        _server_ip=_net_tree_wid->topLevelItem(0)->child(1)->text(2);
    }
    
    _server_port=_net_tree_wid->topLevelItem(0)->child(1)->text(3);
    QString client_port=_server_port;
    if(_server_protocol=="udp"){
        client_port=QString::number(_server_port.toInt()-1);
    }
    _net_tree_wid->topLevelItem(0)->child(0)->setText(HOSTPORT_COL,client_port);
    
    _net_tree_wid->closePersistentEditor(_net_item,_net_column); // close old editor
    _net_item = nullptr;
    _net_column=-1;
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

void EcGuiNet::OnMouseClicked(QTreeWidgetItem* item, int column)
{
    if(_net_item != nullptr){
        _net_tree_wid->closePersistentEditor(_net_item,_net_column); // close old editor
    }

    _net_item = item;
    _net_column=column;

    if((item->text(0)=="Server") &&
       ((column == HOSTNAME_COL) || 
        (column == HOSTIP_COL)   ||
        (column == HOSTPORT_COL))){
        _net_tree_wid->openPersistentEditor(item,column);
    }
    else{
        _net_tree_wid->closePersistentEditor(item,column);
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
}

void EcGuiNet::server_processFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    server_readyStdO();
}

void EcGuiNet::kill_view_process(const QString& terminal_pid)
{
    QStringList cmd={"-9"};
    cmd.append(terminal_pid);
    QProcess kill_view_proc;
    kill_view_proc.start("kill",cmd);
    kill_view_proc.waitForFinished();
}

void EcGuiNet::view_process(const QString &file_path,QString &terminal_pid)
{
    if(terminal_pid!=""){
        kill_view_process(terminal_pid);
        terminal_pid="";
    }

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
    _ssh_command.clear();
    _ssh_command.append("-p");

    _ssh_command.append(_server_pwd);
    _ssh_command.append("ssh");
    _ssh_command.append("-o StrictHostKeyChecking=no");
    _ssh_command.append(_server_hostname+"@"+_server_ip);

    QStringList cmd = _ssh_command;
    cmd.append("'whoami'");
    process->start("sshpass", cmd);
    process->waitForFinished();

    auto host_name = stdout.remove(QChar('\n'));
    
    if(host_name != _server_hostname){
        QMessageBox msgBox;
        msgBox.setText("Problem on the ssh command, please verify the EtherCAT system setup");
        msgBox.exec();
        return false;
    }

    return true;
}

bool EcGuiNet::start_network()
{
    if(!create_ssh_cmd(_ec_master_process,_ec_master_stdout)){
        return false;
    }

    QString bin_file_name = "'repl'";
    kill_process(_ec_master_process,bin_file_name,_ec_master_stdout);
    auto bin_file_path = find_process(_ec_master_process,bin_file_name,_ec_master_stdout);

    /******************************START EtherCAT Master ************************************************/
    if(!bin_file_path.isEmpty()){
        QString option="";
        if(_server_protocol=="udp"){
            option="-f ~/.ecat_master/configs/zipc_config.yaml";  
        }
        start_process(_ec_master_process,bin_file_path,option);

        QFile(_ec_master_file_path).remove();
        _ec_master_file=new QFile(_ec_master_file_path);
        if (_ec_master_file->open(QFile::WriteOnly | QFile::Truncate)) {
            _ec_master_stream=new QTextStream(_ec_master_file);
            view_master_process();
        }
    }
    
    /******************************START SEVER ************************************************/
    if(_server_protocol=="udp") {
        bin_file_name = "'udp_server'";
        kill_process(_server_process,bin_file_name,_server_stdout);
        
        bin_file_path.clear();
        bin_file_path = find_process(_server_process,bin_file_name,_server_stdout);

        if(!bin_file_path.isEmpty()){
            start_process(_server_process,bin_file_path,"");
        }
        QFile(_server_file_path).remove();
        _server_file=new QFile(_server_file_path);
        if (_server_file->open(QFile::WriteOnly | QFile::Truncate)) {
            _server_stream=new QTextStream(_server_file);
            view_server_process();
        }
    }

    return true;
}

void EcGuiNet::stop_network()
{
    QString bin_file_name;
    /******************************STOP Server ************************************************/
    if(_server_protocol=="udp"){
        if(_server_file){
            if(_server_file->isOpen()){
                _server_file->close();
            }
        }
        if(_server_process->state()!=QProcess::NotRunning){
            _server_process->close();
            bin_file_name = "'udp_server'";
            kill_process(_server_process,bin_file_name,_server_stdout);
            kill_view_process(_server_terminal_pid);
        }
    }
    /******************************STOP EtherCAT Master ************************************************/
    if(_ec_master_file){
        if(_ec_master_file->isOpen()){
            _ec_master_file->close();
        }
    }
    if(_ec_master_process->state()!=QProcess::NotRunning){
        _ec_master_process->close();
        bin_file_name = "'repl'";
        kill_process(_ec_master_process,bin_file_name,_ec_master_stdout);
        kill_view_process(_master_terminal_pid);
    }

    kill_view_process(_gui_terminal_pid);
}

EcGuiNet::ec_net_info_t EcGuiNet::get_net_setup()
{
    ec_net_info_t ec_net_info;

    ec_net_info.protocol=_server_protocol.toStdString();
    ec_net_info.host_name=_server_ip.toStdString();
    ec_net_info.host_port=_server_port.toUInt();
    
    return ec_net_info;
}

EcGuiNet::~EcGuiNet()
{
    stop_network();
}
