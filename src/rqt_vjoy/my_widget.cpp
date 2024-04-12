/**
    @author Kenta Suzuki
*/

#include "rqt_vjoy/my_widget.h"

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <QBoxLayout>
#include <QCheckBox>
#include <QFormLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPalette>
#include <QSpinBox>
#include <QTimer>
#include <QToolButton>
#include <QVector>

namespace {

struct ButtonInfo {
    const QString label;
    int row;
    int column;
    bool is_axis;
    double active_value;
    int id;
    int key;
};

ButtonInfo buttonInfo[] = {

    { "E", 3, 3, true, -1.0, 1, Qt::Key_E },
    { "D", 4, 3, true,  1.0, 1, Qt::Key_D },
    { "S", 4, 2, true, -1.0, 0, Qt::Key_S },
    { "F", 4, 4, true,  1.0, 0, Qt::Key_F },

    { "I", 3, 8, true, -1.0, 4, Qt::Key_I },
    { "K", 4, 8, true,  1.0, 4, Qt::Key_K },
    { "J", 4, 7, true, -1.0, 3, Qt::Key_J },
    { "L", 4, 9, true,  1.0, 3, Qt::Key_L },

    { "^", 0, 1, true, -1.0, 7, Qt::Key_Up },
    { "v", 2, 1, true,  1.0, 7, Qt::Key_Down },
    { "<", 1, 0, true, -1.0, 6, Qt::Key_Left },
    { ">", 1, 2, true,  1.0, 6, Qt::Key_Right },

    { "W", 3, 2, true, 1.0, 2, Qt::Key_W },
    { "O", 3, 9, true, 1.0, 5, Qt::Key_O },

    { "A", 2, 10, false, 1.0, 0, Qt::Key_A },
    { "B", 1, 11, false, 1.0, 1, Qt::Key_B },
    { "X", 1,  9, false, 1.0, 2, Qt::Key_X },
    { "Y", 0, 10, false, 1.0, 3, Qt::Key_Y },

    { "R", 3, 4, false, 1.0, 4, Qt::Key_R },
    { "U", 3, 7, false, 1.0, 5, Qt::Key_U },
    { "G", 4, 5, false, 1.0, 6, Qt::Key_G },
    { "H", 4, 6, false, 1.0, 7, Qt::Key_H },

    { "C", 5, 4, false, 1.0,  9, Qt::Key_C },
    { "M", 5, 7, false, 1.0, 10, Qt::Key_M },
    { " ", 5, 5, false, 1.0,  8, Qt::Key_Space }

};

}

namespace rqt_vjoy {

class MyWidget::Impl
{
public:
    MyWidget* self;

    Impl(MyWidget* self);

    void on_toolButton_pressed(int arg1);
    void on_toolButton_released(int arg1);
    void on_publishButton_toggled(bool checked);
    void on_timer_timeout();

    enum {
        Axis_LeftUp, Axis_LeftDown, Axis_LeftLeft, Axis_LeftRight,
        Axis_RightUp, Axis_RightDown, Axis_RightLeft, Axis_RightRight,
        Axis_DirectionalPadUp, Axis_DirectionalPadDown, Axis_DirectionalPadLeft, Axis_DirectionalPadRight,
        Axis_LeftTrigger, Axis_RightTrigger,
        Button_A, Button_B, Button_X, Button_y,
        Button_L, Button_R, Button_Option, Button_Share,
        Button_Logo, Button_LeftStick, Button_RightStick,
        NumJoystickElements
    };

    QToolButton buttons[NumJoystickElements];
    QSpinBox* rateSpin;
    QCheckBox* triggerCheck;
    QToolButton* publishButton;
    QTimer* timer;
    QVector<double> axisPositions;
    QVector<bool> buttonStates;

    ros::NodeHandle n;
    ros::Publisher joy_pub;
};

MyWidget::MyWidget(QWidget* parent)
    : QWidget(parent)
    , JoystickInterface(parent)
{
    impl = new Impl(this);
}

MyWidget::Impl::Impl(MyWidget* self)
    : self(self)
{
    self->setWindowTitle("Virtual Joystick");
    self->setFocusPolicy(Qt::WheelFocus);

    joy_pub = n.advertise<sensor_msgs::Joy>("joy", 1);

    axisPositions.fill(0.0, 8);
    buttonStates.fill(0.0, 11);

    rateSpin = new QSpinBox;
    rateSpin->setRange(0, 1000);
    rateSpin->setValue(0);

    triggerCheck = new QCheckBox;
    triggerCheck->setText("Default trigger value");

    publishButton = new QToolButton;
    publishButton->setIcon(QIcon::fromTheme("network-wireless"));
    publishButton->setToolTip("publish the joy topic");
    publishButton->setCheckable(true);
    self->QWidget::connect(publishButton, &QToolButton::toggled,
        [&](bool checked){ on_publishButton_toggled(checked); });

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(new QLabel("Autorepeat rate"));
    layout2->addWidget(rateSpin);
    layout2->addWidget(new QLabel("Hz"));
    layout2->addWidget(triggerCheck);
    layout2->addWidget(publishButton);
    layout2->addStretch();

    auto gridLayout = new QGridLayout;
    for(int i = 0; i < NumJoystickElements; ++i) {
        ButtonInfo& info = buttonInfo[i];
        QToolButton& button = buttons[i];
        button.setText(info.label);
        button.setFixedSize(24, 24);
        self->QWidget::connect(&button, &QToolButton::pressed, [=](){ on_toolButton_pressed(i); });
        self->QWidget::connect(&button, &QToolButton::released, [=](){ on_toolButton_released(i); });
        gridLayout->addWidget(&button, info.row, info.column);
    }

    QWidget* self2 = static_cast<QWidget*>(self);
    timer = new QTimer(self2);
    self->QWidget::connect(timer, &QTimer::timeout, [&](){ on_timer_timeout(); });

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addLayout(gridLayout);
    layout->addStretch();
    self->setLayout(layout);    
}

MyWidget::~MyWidget()
{
    delete impl;
}

bool MyWidget::ready()
{
    show();
    return true;
}

void MyWidget::read_current_state()
{

}

float MyWidget::axis(const int& id)
{
    return impl->axisPositions[id];
}

bool MyWidget::button(const int& id)
{
    return impl->buttonStates[id];
}

int MyWidget::num_axes()
{
    return impl->axisPositions.size();
}

int MyWidget::num_buttons()
{
    return impl->buttonStates.size();
}

void MyWidget::Impl::on_toolButton_pressed(int arg1)
{
    ButtonInfo& info = buttonInfo[arg1];
    if(info.is_axis) {
        axisPositions[info.id] = info.active_value;
    } else {
        buttonStates[info.id] = info.active_value > 0.0 ? true : false;
    }

    QPalette palette;
    palette.setColor(QPalette::Button, Qt::red);
    buttons[arg1].setPalette(palette);
}

void MyWidget::Impl::on_toolButton_released(int arg1)
{
    ButtonInfo& info = buttonInfo[arg1];
    if(info.is_axis) {
        if(arg1 == 12 || arg1 == 13) {
            axisPositions[info.id] = -1.0;
        } else {
            axisPositions[info.id] = 0.0;
        }
    } else {
        buttonStates[info.id] = false;
    }

    QPalette palette;
    buttons[arg1].setPalette(palette);
}

void MyWidget::Impl::on_publishButton_toggled(bool checked)
{
    if(checked) {
        int rate = rateSpin->value();
        if(rate == 0) {
            timer->start(1);
        } else {
            timer->start(1000 / rate);
        }
    } else {
        timer->stop();
    }
}

void MyWidget::Impl::on_timer_timeout()
{
    sensor_msgs::Joy joy_msg;
    joy_msg.header.stamp = ros::Time().now();

    self->read_current_state();
    joy_msg.axes.resize(self->num_axes());
    joy_msg.buttons.resize(self->num_buttons());

    for(int i = 0; i < self->num_axes(); ++i) {
        joy_msg.axes[i] = self->axis(i);
    }
    for(int i = 0; i < self->num_buttons(); ++i) {
        joy_msg.buttons[i] = self->button(i);
    }

    if(triggerCheck->isChecked()) {
        joy_msg.axes[2] = joy_msg.axes[2] == -1 ? 0 : joy_msg.axes[2];
        joy_msg.axes[5] = joy_msg.axes[5] == -1 ? 0 : joy_msg.axes[5];
    }
    joy_pub.publish(joy_msg);
}

void MyWidget::keyPressEvent(QKeyEvent* event)
{
    for(int i = 0; i < MyWidget::Impl::NumJoystickElements; ++i) {
        ButtonInfo& info = buttonInfo[i];
        if(info.key == event->key()) {
            impl->on_toolButton_pressed(i);
        }
    }
}

void MyWidget::keyReleaseEvent(QKeyEvent* event)
{
    for(int i = 0; i < MyWidget::Impl::NumJoystickElements; ++i) {
        ButtonInfo& info = buttonInfo[i];
        if(info.key == event->key()) {
            impl->on_toolButton_released(i);
        }
    }
}

}
