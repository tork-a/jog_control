#include <boost/thread.hpp>
#include <rviz/config.h>
#include <rviz/visualization_manager.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>
#include <QTimer>
#include "jog_joint_panel.h"
#include "jog_msgs/JogJoint.h"

namespace jog_controller
{

JogJointPanel::JogJointPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  controller_name_ = "jog_joint_node";
  // Parameters
  ros::NodeHandle nh;

  nh.getParam("/" + controller_name_ + "/joint_names", joint_name_);
  jog_joint_num_ = joint_name_.size();
  jog_value_.resize(jog_joint_num_);

  //ros::param::param<std::vector<std::string> >(, joint_name_);

  // ROS_INFO_STREAM("joint_name_:" << joint_name_);
  
  QHBoxLayout* enable_layout = new QHBoxLayout;
  enable_layout->addWidget( new QLabel( "Jog Enable:" ));
  jog_button_ = new QPushButton("OFF");
  jog_button_->setStyleSheet("QPushButton:checked { background-color: red; }\n");
  jog_button_->setCheckable(true);
  jog_button_->setChecked(false);
  enable_layout->addWidget(jog_button_);

  QVBoxLayout* jog_layout = new QVBoxLayout;

  joint_label_.resize(jog_joint_num_);
  jog_slider_.resize(jog_joint_num_);
  for (int i=0; i<jog_joint_num_; i++)
  {
    QHBoxLayout* layout = new QHBoxLayout;

    joint_label_[i] = new QLabel;
    joint_label_[i]->setText(QString::fromStdString(joint_name_[i]));
    layout->addWidget(joint_label_[i]);
    
    jog_slider_[i] = new JogSlider(Qt::Horizontal);
    jog_slider_[i]->setTickPosition(QSlider::TicksBelow);
    jog_slider_[i]->setTickInterval(500);
    jog_slider_[i]->setMinimum(-1000);
    jog_slider_[i]->setMaximum( 1000);
    jog_slider_[i]->setTracking(true);
    jog_slider_[i]->setSingleStep(0);
    jog_slider_[i]->setPageStep(0);
    // How can I change the slider style more properly??
    jog_slider_[i]->setStyleSheet("QSlider::handle {"
                                  "background: white;"
                                  "border: 3px solid black;"
                                  "width: 60px;"
                                  "margin: -30px 0;"
                                  "} "
                                  "QSlider::sub-page {"
                                  "background: rgb(164, 192, 2);"
                                  "} "
                                  "QSlider::add-page {"
                                  "background: rgb(223, 70, 70);"
                                  "} ");
    layout->addWidget(jog_slider_[i]);

    jog_layout->addLayout(layout);
  }

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(enable_layout);
  layout->addLayout(jog_layout);
  setLayout(layout);

  connect(jog_button_, SIGNAL(toggled(bool)), this, SLOT(respondEnable(bool)));

  // Slider
  for (int i=0; i<jog_slider_.size(); i++)
  {
    connect(jog_slider_[i],
            SIGNAL(sliderReleased()), jog_slider_[i],
            SLOT(respondSliderReleased()));
  }
  
  // Timer for publish
  QTimer* output_timer = new QTimer( this );
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( publish() ));
  output_timer->start(100);

  jog_joint_pub_ = nh.advertise<jog_msgs::JogJoint>( "jog_joint", 1);
}

void JogJointPanel::onInitialize()
{
  connect( vis_manager_, SIGNAL( preUpdate() ), this, SLOT( update() ));  
  updateJoint();
}

void JogJointPanel::update()
{
}

void JogJointPanel::updateJoint()
{
}

void JogJointPanel::publish()
{
  // publish
  jog_msgs::JogJoint msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;

  msg.joint_names.resize(jog_joint_num_);
  for (int i=0; i<jog_joint_num_; i++)
  {
    msg.joint_names[i] = joint_label_[i]->text().toUtf8().constData();
  }
    
  msg.deltas.resize(jog_slider_.size());
  for (int i=0; i<jog_slider_.size(); i++)
  {
    msg.deltas[i] = 0.1 * jog_slider_[i]->value() / jog_slider_[i]->maximum();
  }
  // Publish if the button is enabled
  if(jog_button_->isChecked())
  {
    // Publish only if the all value are not equal zero
    bool flag = false;
    for (int i=0; i<jog_slider_.size(); i++)
    {
      if (jog_slider_[i]->value() != 0)
      {
        flag = true;
        break;
      }
    }
    if (flag)
    {
      jog_joint_pub_.publish(msg);
    }
  }
}  

void JogJointPanel::respondEnable(bool checked)
{
  if (checked)
  {
    jog_button_->setText("ON");
  }
  else
  {
    jog_button_->setText("OFF");
  }
}

void JogJointPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void JogJointPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

QLineEdit* JogJointPanel::makeNumericLabel()
{
  QLineEdit* label = new QLineEdit;
  label->setReadOnly( true );
  return label;
}

void JogJointPanel::fillNumericLabel( QLineEdit* label, double value )
{
  label->setText( QString::number( value, 'f', 2 ));
}

void JogSlider::respondSliderReleased()
{
  this->setValue(0);
}

}  // namespace jog_controller


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jog_controller::JogJointPanel, rviz::Panel)
