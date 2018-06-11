#ifndef JOG_FRAME_PANEL_H
#define JOG_FRAME_PANEL_H

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

namespace jog_controller
{
class JogFramePanel: public rviz::Panel
  {
  Q_OBJECT
  public:
    JogFramePanel(QWidget* parent = 0);
    
    virtual void onInitialize();
    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

  protected Q_SLOTS:
    void update();
    void updateFrame();
    void updateGroups();
    void updateTargetLink();
    void respondEnable(bool checked);
    void respondFrame(int index);
    void respondTargetLink(int index);
    void respondAxis(int index);
    void respondOrientationAxis(int index);
    void respondSliderChanged(int value);
    void respondSliderReleased();
    void respondOrientationSliderChanged(int value);
    void respondOrientationSliderReleased();
    void publish();
    
  protected:
    QPushButton* jog_button_;
    QComboBox* group_cbox_;
    QComboBox* frame_cbox_;
    QComboBox* target_link_cbox_;
    QComboBox* axis_cbox_;
    QComboBox* ori_axis_cbox_;
    QSlider* jog_slider_;
    QSlider* ori_jog_slider_;
    QLineEdit* pos_x_text_;
    QLineEdit* pos_y_text_;
    QLineEdit* pos_z_text_;
    QLineEdit* rot_x_text_;
    QLineEdit* rot_y_text_;
    QLineEdit* rot_z_text_;
    std::vector<std::string> group_names_;
    std::vector<std::string> link_names_;
    std::string frame_id_;
    std::string target_link_id_;
    std::string axis_id_;
    std::string ori_axis_id_;
    boost::mutex mutex_;
    double jog_value_;
    double ori_jog_value_;
    ros::Publisher jog_frame_pub_;

    void initFrameComboBox();
    void initAxisComboBox();
    void initOrientationAxisComboBox();

    QLineEdit* makeNumericLabel();
    void fillNumericLabel( QLineEdit* label, double value );
  };

}  // namespace jog_controller

#endif  // JOG_FRAME_PANEL_H
