#ifndef JOG_SLIDER_H
#define JOG_SLIDER_H

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

#include <QSlider>

namespace jog_controller
{
class JogSlider : public QSlider
{
  Q_OBJECT

public:
  explicit JogSlider(Qt::Orientation orientation)
    : QSlider(orientation)  {}
  
public Q_SLOTS:
  void respondSliderReleased();
};

}  // namespace jog_controller

#endif  // JOG_SLIDER
