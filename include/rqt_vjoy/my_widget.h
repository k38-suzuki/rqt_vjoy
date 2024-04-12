/**
    @author Kenta Suzuki
*/

#ifndef rqt_vjoy__my_widget_H
#define rqt_vjoy__my_widget_H

#include <QWidget>

#include "rqt_vjoy/joystick_interface.h"

namespace rqt_vjoy {

class MyWidget : public QWidget, public JoystickInterface
{
public:
    MyWidget(QWidget* parent = nullptr);

    virtual ~MyWidget();

    virtual bool ready() override;
    virtual void read_current_state() override;
    virtual float axis(const int& id) override;
    virtual bool button(const int& id) override;
    virtual int num_axes() override;
    virtual int num_buttons() override;

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void keyReleaseEvent(QKeyEvent* event) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif // rqt_vjoy__my_widget_H
