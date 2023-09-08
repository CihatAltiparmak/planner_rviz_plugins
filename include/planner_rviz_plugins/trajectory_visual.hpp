#ifndef __TRAJECTORY_VISUAL_HPP__
#define __TRAJECTORY_VISUAL_HPP__

#include <Ogre.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "rviz_rendering/objects/shape.hpp"

namespace Ogre {
class Quaternion;
}

namespace rviz_rendering {
class Shape;
}

namespace planner_rviz_plugins {

class TrajectoryVisual {
   public:
    TrajectoryVisual(Ogre::SceneManager*, Ogre::SceneNode*);
    virtual ~TrajectoryVisual();

    void setMessage(geometry_msgs::msg::PoseStamped::ConstSharedPtr);
    void setFramePosition(const Ogre::Vector3&);
    void setFrameOrientation(const Ogre::Quaternion&);
    void setColor(float, float, float, float);

   private:
    std::shared_ptr<rviz_rendering::Shape> pose_shape_;
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;
};

}  // end of namespace planner_rviz_plugins

#endif