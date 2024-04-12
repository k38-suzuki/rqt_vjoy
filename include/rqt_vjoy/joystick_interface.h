/**
   @author Kenta Suzuki
*/

#ifndef rqt_vjoy__joystick_interface_H
#define rqt_vjoy__joystick_interface_H

#include <QObject>

namespace rqt_vjoy {

class JoystickInterface : public QObject
{
    Q_OBJECT
public:
    JoystickInterface(QObject* parent = nullptr);
    ~JoystickInterface();

    virtual bool ready() = 0;
    virtual void read_current_state() = 0;
    virtual float axis(const int& axisId) = 0;
    virtual bool button(const int& buttonId) = 0;
    virtual int num_axes() = 0;
    virtual int num_buttons() = 0;
};

}

#endif // rqt_vjoy__joystick_interface_H
