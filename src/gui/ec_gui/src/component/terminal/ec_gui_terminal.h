#ifndef EC_GUI_TERMINAL_H
#define EC_GUI_TERMINAL_H


#include <QtUiTools/QtUiTools>
#include <QWidget>

class EcGuiTerminal : public QWidget
{
Q_OBJECT
public:

    typedef std::shared_ptr<EcGuiTerminal> Ptr;
    explicit EcGuiTerminal(QWidget * parent = 0);

    void setText(QString terminal_line);

    ~EcGuiTerminal();

private:

    QTextBrowser* _terminal_txt;

};

#endif // EC_GUI_TERMINAL_H
