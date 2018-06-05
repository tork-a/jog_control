#include <boost/thread.hpp>
#include <rviz/config.h>
#include <rviz/visualization_manager.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>
#include <QTimer>
#include "jog_frame_panel.h"
#include "jog_msgs/JogFrame.h"

namespace jog_controller
{

JogFramePanel::JogFramePanel(QWidget* parent)
  : rviz::Panel(parent), jog_value_(0)
{
  ros::NodeHandle nh;
  jog_frame_pub_ = nh.advertise<jog_msgs::JogFrame>( "jog_frame", 1);

  // Get groups parameter of jog_frame_node
  nh.getParam("/jog_frame_node/group_names", group_names_);
  for (int i=0; i<group_names_.size(); i++)
  {
    ROS_INFO_STREAM("group_names:" << group_names_[i]);
  }
  nh.getParam("/jog_frame_node/link_names", link_names_);
  for (int i=0; i<link_names_.size(); i++)
  {
    ROS_INFO_STREAM("link_names:" << link_names_[i]);
  }
  
  QHBoxLayout* enable_layout = new QHBoxLayout;
  enable_layout->addWidget( new QLabel( "Jog Enable:" ));
  jog_button_ = new QPushButton("OFF");
  jog_button_->setStyleSheet("QPushButton:checked { background-color: red; }\n");
  jog_button_->setCheckable(true);
  jog_button_->setChecked(false);
  enable_layout->addWidget(jog_button_);

  QHBoxLayout* group_layout = new QHBoxLayout;
  group_layout->addWidget( new QLabel( "Group:" ));
  group_cbox_ = new QComboBox();
  updateGroups();
  group_layout->addWidget(group_cbox_);

  QHBoxLayout* frame_layout = new QHBoxLayout;
  frame_layout->addWidget( new QLabel( "Frame:" ));
  frame_cbox_ = new QComboBox();
  frame_layout->addWidget(frame_cbox_);

  QHBoxLayout* target_layout = new QHBoxLayout;
  target_layout->addWidget( new QLabel( "Target:" ));
  target_link_cbox_ = new QComboBox();
  target_layout->addWidget(target_link_cbox_);

  QHBoxLayout* axis_layout = new QHBoxLayout;
  axis_layout->addWidget( new QLabel( "Axis:" ));
  axis_cbox_ = new QComboBox();
  axis_layout->addWidget(axis_cbox_);

  QHBoxLayout* jog_layout = new QHBoxLayout;
  jog_layout->addWidget( new QLabel( "Jog:" ));
  jog_slider_ = new QSlider(Qt::Horizontal);
  jog_slider_->setTickPosition(QSlider::TicksBelow);
  jog_slider_->setTickInterval(500);
  jog_slider_->setMinimum(-1000);
  jog_slider_->setMaximum( 1000);
  jog_slider_->setTracking(true);
  jog_slider_->setSingleStep(0);
  jog_slider_->setPageStep(0);
  // How can I change the slider style more properly??
  jog_slider_->setStyleSheet("QSlider::handle {"
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
  jog_layout->addWidget(jog_slider_);

  QHBoxLayout* pos_x_layout = new QHBoxLayout;
  pos_x_layout->addWidget( new QLabel( "X:" ));
  pos_x_text_ = makeNumericLabel();
  pos_x_layout->addWidget(pos_x_text_);

  QHBoxLayout* pos_y_layout = new QHBoxLayout;
  pos_y_layout->addWidget( new QLabel( "Y:" ));
  pos_y_text_ = makeNumericLabel();
  pos_y_layout->addWidget(pos_y_text_);

  QHBoxLayout* pos_z_layout = new QHBoxLayout;
  pos_z_layout->addWidget( new QLabel( "Z:" ));
  pos_z_text_ = makeNumericLabel();
  pos_z_layout->addWidget(pos_z_text_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(enable_layout);
  layout->addLayout(group_layout);
  layout->addLayout(frame_layout);
  layout->addLayout(target_layout);
  layout->addLayout(axis_layout);
  layout->addLayout(jog_layout);
  layout->addLayout(pos_x_layout);
  layout->addLayout(pos_y_layout);
  layout->addLayout(pos_z_layout);
  setLayout(layout);

  connect(jog_button_, SIGNAL(toggled(bool)), this, SLOT(respondEnable(bool)));
  connect(frame_cbox_, SIGNAL(activated(int)), this, SLOT(respondFrame(int)));
  connect(target_link_cbox_, SIGNAL(activated(int)), this, SLOT(respondTargetLink(int)));
  connect(axis_cbox_, SIGNAL(activated(int)), this, SLOT(respondAxis(int)));

  // Slider
  connect(jog_slider_, SIGNAL(valueChanged(int)), this, SLOT(respondSliderChanged(int)));
  connect(jog_slider_, SIGNAL(sliderReleased()), this, SLOT(respondSliderReleased()));

  // Timer for update Frame ComboBox
  QTimer* output_timer = new QTimer( this );
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( publish() ));
  output_timer->start(10);
}

void JogFramePanel::onInitialize()
{
  connect( vis_manager_, SIGNAL( preUpdate() ), this, SLOT( update() ));  
  updateFrame();
  updateTargetLink();
  initAxisComboBox();
}

void JogFramePanel::update()
{
  tf::TransformListener* tf = vis_manager_->getTFClient();
  tf::StampedTransform transform;
  try{
    tf->lookupTransform(frame_id_, target_link_id_, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }  
  fillNumericLabel(pos_x_text_, transform.getOrigin().x());
  fillNumericLabel(pos_y_text_, transform.getOrigin().y());
  fillNumericLabel(pos_z_text_, transform.getOrigin().z());
}

void JogFramePanel::updateGroups()
{
  group_cbox_->clear();
  for (auto it = group_names_.begin(); it != group_names_.end(); it++)
  {
    const std::string& group = *it;
    if (group.empty())
    {
      continue;
    }
    group_cbox_->addItem(group.c_str());
  }
}


void JogFramePanel::updateFrame()
{
  typedef std::vector<std::string> V_string;
  V_string frames;
  vis_manager_->getTFClient()->getFrameStrings( frames );
  std::sort(frames.begin(), frames.end());
  frame_cbox_->clear();
  for (V_string::iterator it = frames.begin(); it != frames.end(); ++it )
  {
    const std::string& frame = *it;
    if (frame.empty())
    {
      continue;
    }
    frame_cbox_->addItem(it->c_str());
  }
  frame_cbox_->setCurrentIndex(0);
  frame_id_ = frame_cbox_->currentText().toStdString();
}

void JogFramePanel::updateTargetLink()
{
  target_link_cbox_->clear();
  for (int i=0; i<link_names_.size(); i++)
  {
    target_link_cbox_->addItem(link_names_[i].c_str());
  }
  target_link_cbox_->setCurrentIndex(0);
  target_link_id_ = target_link_cbox_->currentText().toStdString();
}


void JogFramePanel::publish()
{
  boost::mutex::scoped_lock lock(mutex_);
  
  // publish
  jog_msgs::JogFrame msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  msg.group_name = group_cbox_->currentText().toStdString();
  msg.link_name = target_link_id_;
  
  if (axis_id_ == "x")
  {
    msg.linear.x = jog_value_;
  }
  if (axis_id_ == "y")
  {
    msg.linear.y = jog_value_;
  }
  if (axis_id_ == "z")
  {
    msg.linear.z = jog_value_;
  }

  if(jog_button_->isChecked())
  {
    jog_frame_pub_.publish(msg);
  }
}  

void JogFramePanel::respondEnable(bool checked)
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

void JogFramePanel::respondFrame(int index)
{
  boost::mutex::scoped_lock lock(mutex_);
  frame_id_ = frame_cbox_->currentText().toStdString();
  ROS_INFO_STREAM("respondFrame: " << frame_id_);
}

void JogFramePanel::respondTargetLink(int index)
{
  boost::mutex::scoped_lock lock(mutex_);
  target_link_id_ = target_link_cbox_->currentText().toStdString();
  ROS_INFO_STREAM("respondTargetLink: " << target_link_id_);
}

void JogFramePanel::respondAxis(int index)
{
  boost::mutex::scoped_lock lock(mutex_);
  axis_id_ = axis_cbox_->currentText().toStdString();
  ROS_INFO_STREAM("respondAxis: " << axis_id_);
}

void JogFramePanel::respondSliderChanged(int value)
{
  boost::mutex::scoped_lock lock(mutex_);
  jog_value_ = 0.05 * value / 1000.0;
}

void JogFramePanel::respondSliderReleased()
{
  jog_slider_->setValue(0);
}

void JogFramePanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void JogFramePanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

void JogFramePanel::initAxisComboBox()
{
  axis_cbox_->addItem("x");
  axis_cbox_->addItem("y");
  axis_cbox_->addItem("z");
  axis_cbox_->setCurrentIndex(0);
  axis_id_ = axis_cbox_->currentText().toStdString();
}

QLineEdit* JogFramePanel::makeNumericLabel()
{
  QLineEdit* label = new QLineEdit;
  label->setReadOnly( true );
  return label;
}

void JogFramePanel::fillNumericLabel( QLineEdit* label, double value )
{
  label->setText( QString::number( value, 'f', 2 ));
}

}  // namespace jog_controller


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jog_controller::JogFramePanel, rviz::Panel)
