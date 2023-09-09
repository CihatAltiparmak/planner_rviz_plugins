#ifndef __TRAJECTORY_POINT_VISUAL_HPP__
#define __TRAJECTORY_POINT_VISUAL_HPP__

#include <Ogre.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <planner_msgs/msg/point.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace Ogre {
class Quaternion;
}

namespace rviz_rendering {
class Shape;
}

namespace planner_rviz_plugins {

class TrajectoryPointVisual {
   public:
    TrajectoryPointVisual(Ogre::SceneManager*, Ogre::SceneNode*);
    virtual ~TrajectoryPointVisual();

    void setMessage(planner_msgs::msg::Point::ConstSharedPtr);
    void setFramePosition(const Ogre::Vector3&);
    void setFrameOrientation(const Ogre::Quaternion&);
    void setColor(float, float, float, float);
    void setDimension(const Ogre::Vector3&);

   private:
    std::shared_ptr<rviz_rendering::Shape> pose_shape_;
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;
};

}  // end of namespace planner_rviz_plugins

#endif