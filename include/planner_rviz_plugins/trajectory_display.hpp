#ifndef __TRAJECTORY_DISPLAY_HPP__
#define __TRAJECTORY_DISPLAY_HPP__

#ifndef Q_MOC_RUN

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>

#include <deque>
#include <memory>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/vector_property.hpp>
#include <rviz_common/visualization_manager.hpp>

#include "rviz_common/message_filter_display.hpp"
// #include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>

#include <planner_msgs/msg/path.hpp>
#include <planner_rviz_plugins/trajectory_visual.hpp>

#endif

namespace Ogre {
class SceneNode;
}

namespace rviz_common {
namespace properties {
class ColorProperty;
class FloatProperty;
class IntProperty;
class VectorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace planner_rviz_plugins {

class TrajectoryPointVisual;
class TrajectoryVisual;

class TrajectoryDisplay
    : public rviz_common::MessageFilterDisplay<planner_msgs::msg::Path> {
    Q_OBJECT

   public:
    TrajectoryDisplay();
    virtual ~TrajectoryDisplay();

   protected:
    virtual void onInitialize();
    virtual void reset();

   private Q_SLOTS:
    void updateColorAndAlpha();
    void updateHistoryLength();
    void updateCarDimension();

   private:
    void processMessage(planner_msgs::msg::Path::ConstSharedPtr msg);

    std::deque<std::shared_ptr<TrajectoryVisual>> visuals_;

    rviz_common::properties::ColorProperty* color_property_;
    rviz_common::properties::FloatProperty* alpha_property_;
    rviz_common::properties::VectorProperty* car_dimension_property_;
    rviz_common::properties::IntProperty* history_length_property_;
};

}  // end of namespace planner_rviz_plugins

#endif