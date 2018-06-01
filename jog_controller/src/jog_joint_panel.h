#ifndef JOG_JOINT_PANEL_H
#define JOG_JOINT_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <boost/thread.hpp>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#endif

#include "jog_slider.h"

namespace jog_controller
{
class JogJointPanel: public rviz::Panel
  {
  Q_OBJECT
  public:
    JogJointPanel(QWidget* parent = 0);
    
    virtual void onInitialize();
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

  protected Q_SLOTS:
    void update();
    void updateJoint();
    void respondEnable(bool checked);
    void publish();
    
  protected:
    QPushButton* jog_button_;
    std::vector<QLabel*> joint_label_;
    std::vector<JogSlider*> jog_slider_;

    std::string controller_name_;
    std::vector<std::string> joint_name_;
    int jog_joint_num_;
    std::string frame_id_;
    std::vector<double> jog_value_;
    ros::Publisher jog_joint_pub_;

    QLineEdit* makeNumericLabel();
    void fillNumericLabel( QLineEdit* label, double value );
  };

}  // namespace jog_controller

#endif  // JOG_JOINT_PANEL_H
