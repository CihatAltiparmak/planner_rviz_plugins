#ifndef __TRAJECTORY_VISUAL_HPP__
#define __TRAJECTORY_VISUAL_HPP__

#include <Ogre.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <planner_msgs/msg/path.hpp>
#include <planner_rviz_plugins/trajectory_point_visual.hpp>
#include <rviz_rendering/objects/shape.hpp>

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

    void setMessage(planner_msgs::msg::Path::ConstSharedPtr);
    void setFramePosition(const Ogre::Vector3&);
    void setFrameOrientation(const Ogre::Quaternion&);
    void setColor(float, float, float, float);
    void setDimension(const Ogre::Vector3&);

   private:
    std::vector<std::shared_ptr<TrajectoryPointVisual>>
        trajectoryPointVisualList_;
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;
};

}  // end of namespace planner_rviz_plugins

#endif