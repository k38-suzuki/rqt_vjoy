/**
    @author Kenta Suzuki
*/

#ifndef rqt_vjoy__joy_widget_H
#define rqt_vjoy__joy_widget_H

#include <QWidget>

#include "rqt_vjoy/joystick_interface.h"

namespace rqt_vjoy {

class JoyWidget : public QWidget, public JoystickInterface
{
public:
    JoyWidget(QWidget* parent = nullptr);

    virtual ~JoyWidget();

    void setDevice(const QString& device);

    virtual bool ready() override;
    virtual void read_current_state() override;
    virtual float axis(const int& axisId) override;
    virtual bool button(const int& buttonId) override;
    virtual int num_axes() override;
    virtual int num_buttons() override;

    QString identifier() const;

    enum AxisId {
        Axis_LeftHorizontal,
        Axis_LeftVertical,
        Axis_LeftTrigger,
        Axis_RightHorizontal,
        Axis_RightVertical,
        Axis_RightTrigger,
        Axis_DirectionalPadHorizontal,
        Axis_DirectionalPadVertical,
        NumAxes
    };

    enum ButtonId {
        Button_A,
        Button_B,
        Button_X,
        Button_Y,
        Bumper_Left,
        Bumper_Right,
        Button_Back,
        Button_Start,
        Button_Guide,
        Button_LeftStick,
        Button_RightStick,
        NumButtons
    };

private:
    class Impl;
    Impl* impl;
};

}

#endif // rqt_vjoy__joy_widget_H
