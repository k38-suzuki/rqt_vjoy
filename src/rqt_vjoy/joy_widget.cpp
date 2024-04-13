/**
    @author Kenta Suzuki
*/

#include "rqt_vjoy/joy_widget.h"

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include <QBoxLayout>
#include <QCheckBox>
#include <QComboBox>
#include <QDebug>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QSpinBox>
#include <QTimer>
#include <QToolButton>
#include <QVector>

#include <vector>
#include <memory>

#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

namespace {

enum Model { Ps4, Xbox, F310x, Unsupported };

const int ps4_axes[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
const int ps4_buttons[] = { 0, 1, 3, 2, 4, 5, 8, 9, 10, 11, 12 };

const int xbox_axes[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
const int xbox_buttons[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

const int f310x_axes[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
const int f310x_buttons[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

struct ModelInfo {
    Model id;
    const int* axis_map;
    const int* button_map;
};

const QMap<QString, Model> modelMap = {
    { "Sony Interactive Entertainment DUALSHOCK®4 USB Wireless Adaptor", Ps4 },
    { "Microsoft X-Box 360 pad", Xbox },
    { "Logitech Gamepad F310", F310x }
};

const ModelInfo modelInfo[] = {
    { Ps4,         ps4_axes,   ps4_buttons   },
    { Xbox,        xbox_axes,  xbox_buttons  },
    { F310x,       f310x_axes, f310x_buttons },
    { Unsupported, nullptr,    nullptr       }
};

const QStringList buttonList = {
    "A_BUTTON", "B_BUTTON", "X_BUTTON", "Y_BUTTON",
    "LEFT_BUMPER", "RIGHT_BUMPER", "BACK_BUTTON", "START_BUTTON",
    "GUIDE_BUTTON", "LEFT_STICK_BUTTON", "RIGHT_STICK_BUTTON"
};

const QStringList axisList = {
    "LEFT_STICK_H", "LEFT_STICK_V", "LEFT_TRIGGER",
    "RIGHT_STICK_H", "RIGHT_STICK_V", "RIGHT_TRIGGER",
    "D_PAD_H", "D_PAD_V"
};

}

namespace rqt_vjoy {

class JoyWidget::Impl
{
public:
    JoyWidget* self;

    Impl(JoyWidget* self);

    void on_publishButton_toggled(bool checked);
    void on_timer_timeout();

    void read_current_state();

    void setDevice(const QString& device);

    struct AxisUnit
    {
        QLabel* idLabel;
        QComboBox* idCombo;
        QLabel* maxLabel;
        QDoubleSpinBox* maxSpin;
        QCheckBox* reverseCheck;

        ~AxisUnit()
        {
            delete idLabel;
            delete idCombo;
            delete maxLabel;
            delete maxSpin;
            delete reverseCheck;
        }
    };

    struct ButtonUnit
    {
        QLabel* idLabel;
        QComboBox*  idCombo;

        ~ButtonUnit()
        {
            delete idLabel;
            delete idCombo;
        }
    };

    QString identifierName;
    QLineEdit* topicLine;
    QToolButton* publishButton;
    QSpinBox* rateSpin;
    QCheckBox* triggerCheck;
    QLineEdit* deviceLine;
    QDoubleSpinBox* zoneSpin;
    QSpinBox* delaySpin;
    QGridLayout* gridLayout;
    QGridLayout* gridLayout2;
    QTimer* timer;
    QVector<int> axes;
    QVector<char> buttons;
    QVector<sensor_msgs::Joy> buffer;
    std::vector<std::unique_ptr<AxisUnit>> axisUnits;
    std::vector<std::unique_ptr<ButtonUnit>> buttonUnits;

    ros::NodeHandle n;
    ros::Publisher joy_pub;

    bool is_ready;
    int joy_fd;

    ModelInfo currentModel;
};

JoyWidget::JoyWidget(QWidget* parent)
    : QWidget(parent)
    , JoystickInterface(parent)
{
    impl = new Impl(this);
}

JoyWidget::Impl::Impl(JoyWidget* self)
    : self(self)
    , joy_fd(-1)
    , is_ready(false)
{
    self->setWindowTitle("Joystick");

    axisUnits.clear();
    buttonUnits.clear();
    gridLayout = new QGridLayout;
    gridLayout2 = new QGridLayout;
    setDevice("/dev/input/js0");

    topicLine = new QLineEdit;
    topicLine->setText("joy");

    publishButton = new QToolButton;
    publishButton->setIcon(QIcon::fromTheme("network-wireless"));
    publishButton->setToolTip("Publish the joy topic");
    publishButton->setCheckable(true);
    self->QWidget::connect(publishButton, &QToolButton::toggled,
        [&](bool checked){ on_publishButton_toggled(checked); });

    rateSpin = new QSpinBox;
    rateSpin->setRange(0, 1000);
    rateSpin->setValue(0);

    triggerCheck = new QCheckBox;
    triggerCheck->setText("Default trigger value");

    deviceLine = new QLineEdit;
    deviceLine->setText("/dev/input/js0");
    self->QWidget::connect(deviceLine, &QLineEdit::textChanged,
        [&](const QString& text){ setDevice(text); });

    zoneSpin = new QDoubleSpinBox;
    zoneSpin->setRange(0.0, 1.0);
    zoneSpin->setValue(0.05);

    delaySpin = new QSpinBox;
    delaySpin->setRange(0, 10000);
    delaySpin->setValue(0);

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(new QLabel("Topic"));
    layout2->addWidget(topicLine);
    layout2->addWidget(publishButton);
    layout2->addStretch();

    auto layout3 = new QHBoxLayout;
    layout3->addWidget(new QLabel("Autorepeat rate"));
    layout3->addWidget(rateSpin);
    layout3->addWidget(new QLabel("Hz"));
    layout3->addWidget(triggerCheck);
    layout3->addStretch();

    auto layout4 = new QHBoxLayout;
    layout4->addWidget(new QLabel("Device"));
    layout4->addWidget(deviceLine);
    layout4->addWidget(new QLabel("Dead zone"));
    layout4->addWidget(zoneSpin);
    layout4->addWidget(new QLabel("Delay"));
    layout4->addWidget(delaySpin);
    layout4->addWidget(new QLabel("ms"));
    layout4->addStretch();

    timer = new QTimer(static_cast<QWidget*>(self));
    self->QWidget::connect(timer, &QTimer::timeout, [&](){ on_timer_timeout(); });

    auto groupBox = new QGroupBox("Axes");
    {
        auto layout = new QVBoxLayout;
        layout->addLayout(gridLayout);
        layout->addStretch();
        groupBox->setLayout(layout);
    }

    auto groupBox2 = new QGroupBox("Buttons");
    {
        auto layout = new QVBoxLayout;
        layout->addLayout(gridLayout2);
        layout->addStretch();
        groupBox2->setLayout(layout);
    }

    auto layout5 = new QHBoxLayout;
    layout5->addWidget(groupBox);
    layout5->addWidget(groupBox2);
    layout5->addStretch();

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addLayout(layout3);
    layout->addLayout(layout4);
    layout->addLayout(layout5);
    layout->addStretch();
    self->setLayout(layout);
}

JoyWidget::~JoyWidget()
{
    delete impl;
}

void JoyWidget::setDevice(const QString& device)
{
    impl->setDevice(device);
}

void JoyWidget::Impl::setDevice(const QString& device)
{
    axes.clear();
    buttons.clear();

    char name[80];
    int num_axes = 0;
    int num_buttons = 0;

    if(is_ready) {
        ::close(joy_fd);
    }

    joy_fd = open(device.toStdString().c_str(), O_RDONLY);
    is_ready = joy_fd < 0 ? false : true;

    if(is_ready) {
        ioctl(joy_fd, JSIOCGNAME(80), &name);
        ioctl(joy_fd, JSIOCGAXES, &num_axes);
        ioctl(joy_fd, JSIOCGBUTTONS, &num_buttons);

        axes.fill(0, num_axes);
        buttons.fill(0, num_buttons);

        fcntl(joy_fd, F_SETFL, O_NONBLOCK);
        identifierName = name;
    }

    Model id = Unsupported;
    if(modelMap.contains(identifierName)) {
        id = modelMap.value(identifierName);
        qDebug() << identifierName << "is connected.";
    } else {
        qDebug() << "Supported devices not found.";
    }
    currentModel = modelInfo[id];

    axisUnits.clear();
    axisUnits.resize(axes.size());
    QStringList list;
    for(size_t i = 0; i < axisUnits.size(); ++i) {
        list << QString("%1").arg(i);
    }

    for(size_t i = 0; i < axisUnits.size(); ++i) {
        auto& unit = axisUnits[i];
        unit.reset(new AxisUnit);

        unit->idLabel = new QLabel(QString("ID %1").arg(i));
        unit->idCombo = new QComboBox(self);
        if(axisUnits.size() == 8) {
            unit->idCombo->addItems(list);
        } else {
            unit->idCombo->addItems(axisList);
        }
        unit->idCombo->setCurrentIndex(i);
        unit->maxLabel = new QLabel("Max");
        unit->maxSpin = new QDoubleSpinBox(self);
        unit->maxSpin->setRange(0.0, 1.0);
        unit->maxSpin->setValue(1.0);
        unit->reverseCheck = new QCheckBox(self);
        unit->reverseCheck->setText("R");

        gridLayout->addWidget(unit->idLabel, i, 0);
        gridLayout->addWidget(unit->idCombo, i, 1);
        gridLayout->addWidget(unit->maxLabel, i, 2);
        gridLayout->addWidget(unit->maxSpin, i, 3);
        gridLayout->addWidget(unit->reverseCheck, i, 4);
    }

    buttonUnits.clear();
    buttonUnits.resize(buttons.size());
    QStringList list2;
    for(size_t i = 0; i < buttonUnits.size(); ++i) {
        list2 << QString("%1").arg(i);
    }

    for(size_t i = 0; i < buttonUnits.size(); ++i) {
        auto& unit = buttonUnits[i];
        unit.reset(new ButtonUnit);

        unit->idLabel = new QLabel(QString("ID %1").arg(i));
        unit->idCombo = new QComboBox(self);
        if(buttonUnits.size() == 11) {
            unit->idCombo->addItems(buttonList);
        } else {
            unit->idCombo->addItems(list2);
        }
        unit->idCombo->setCurrentIndex(i);

        gridLayout2->addWidget(unit->idLabel, i, 0);
        gridLayout2->addWidget(unit->idCombo, i, 1);
    }
}

bool JoyWidget::ready()
{
    return impl->is_ready;
}

void JoyWidget::read_current_state()
{
    if(impl->is_ready) {
        impl->read_current_state();
    }
}

void JoyWidget::Impl::read_current_state()
{
    if(is_ready) {
        js_event event;
        read(joy_fd, &event, sizeof(js_event));

        switch(event.type & ~JS_EVENT_INIT) {
            case JS_EVENT_AXIS: {
                if((int)event.number >= axes.size()) {
                    // continue;
                    return;
                }
                axes[(int)event.number] = event.value;
                break;
            }
            case JS_EVENT_BUTTON: {
                if((int)event.number >= buttons.size()) {
                    // continue;
                    return;
                }
                buttons[(int)event.number] = event.value;
                break;
            }
        }
    }
}

float JoyWidget::axis(const int& axisId)
{
    int id = axisId;
    if(impl->currentModel.id != Unsupported) {
        id = impl->currentModel.axis_map[axisId];
    }
    return  (float)impl->axes[id] / 32767.0;
}

bool JoyWidget::button(const int& buttonId)
{
    int id = buttonId;
    if(impl->currentModel.id != Unsupported) {
        id = impl->currentModel.button_map[buttonId];
    }
    return impl->buttons[id] ? true : false;
}

int JoyWidget::num_axes()
{
    return impl->axes.size();
}

int JoyWidget::num_buttons()
{
    return impl->buttons.size();
}

QString JoyWidget::identifier() const
{
    return impl->identifierName;
}

void JoyWidget::Impl::on_publishButton_toggled(bool checked)
{
    if(checked) {
        const char* topic_name = topicLine->text().toStdString().c_str();
        joy_pub = n.advertise<sensor_msgs::Joy>(topic_name, 1);

        int rate = rateSpin->value();
        int delay = delaySpin->value();
        if(rate == 0) {
            buffer.clear();
            buffer.resize(delay);
            ROS_INFO("Buffer has been resized %d.", delay);
            timer->start(1);
        } else {
            timer->start(1000 / rate);
        }
    } else {
        joy_pub.shutdown();
        timer->stop();
    }
}

void JoyWidget::Impl::on_timer_timeout()
{
    sensor_msgs::Joy joy_msg;
    joy_msg.header.stamp = ros::Time().now();
    joy_msg.header.frame_id = deviceLine->text().toStdString().c_str();

    self->read_current_state();

    joy_msg.axes.resize(self->num_axes());
    for(int i = 0; i < self->num_axes(); ++i) {
        auto& unit = axisUnits[i];
        int id = unit->idCombo->currentIndex();
        double max = unit->maxSpin->value();
        bool is_reversed = unit->reverseCheck->isChecked();

        joy_msg.axes[i] = self->axis(id);
        double dead_zone = zoneSpin->value();
        if(fabs(joy_msg.axes[i]) < dead_zone) {
            joy_msg.axes[i] = 0.0;
        }
        joy_msg.axes[i] *= max;
        joy_msg.axes[i] *= is_reversed ? -1.0 : 1.0;
    }
    if(triggerCheck->isChecked()) {
        joy_msg.axes[2] = joy_msg.axes[2] == -1 ? 0 : joy_msg.axes[2];
        joy_msg.axes[5] = joy_msg.axes[5] == -1 ? 0 : joy_msg.axes[5];
    }

    joy_msg.buttons.resize(self->num_buttons());
    for(int i = 0; i < self->num_buttons(); ++i) {
        auto& unit = buttonUnits[i];
        int id = unit->idCombo->currentIndex();

        joy_msg.buttons[i] = self->button(id);
    }

    buffer.push_back(joy_msg);
    sensor_msgs::Joy first_msg = buffer.takeFirst();
    joy_pub.publish(first_msg);
}

}
