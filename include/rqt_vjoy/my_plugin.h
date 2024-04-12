/**
    @author Kenta Suzuki
*/

#ifndef rqt_vjoy__my_plugin_H
#define rqt_vjoy__my_plugin_H

#include <rqt_gui_cpp/plugin.h>

namespace rqt_vjoy {

class MyPlugin : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    MyPlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;
    virtual void shutdownPlugin() override;
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

    // Comment in to signal that the plugin has a way to configure it
    // bool hasConfiguration() const;
    // void triggerConfiguration();
};

} // namespace

#endif // rqt_vjoy__my_plugin_H
