#include <planner_rviz_plugins/trajectory_visual.hpp>

namespace planner_rviz_plugins {

TrajectoryVisual::TrajectoryVisual(Ogre::SceneManager* scene_manager,
                                   Ogre::SceneNode* parent_node) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
}

TrajectoryVisual::~TrajectoryVisual() {
    scene_manager_->destroySceneNode(frame_node_);
}

void TrajectoryVisual::setMessage(planner_msgs::msg::Path::ConstSharedPtr msg) {
    trajectoryPointVisualList_.clear();
    for (auto traj_point : msg->points) {
        std::shared_ptr<TrajectoryPointVisual> visual;
        visual.reset(new TrajectoryPointVisual(scene_manager_, frame_node_));
        visual->setMessage(
            std::make_shared<planner_msgs::msg::Point>(traj_point));

        trajectoryPointVisualList_.push_back(visual);
    }
}

void TrajectoryVisual::setFramePosition(const Ogre::Vector3& position) {
    frame_node_->setPosition(position);
}

void TrajectoryVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
    frame_node_->setOrientation(orientation);
}

void TrajectoryVisual::setColor(float r, float g, float b, float a) {
    for (auto point_visual : trajectoryPointVisualList_) {
        point_visual->setColor(r, g, b, a);
    }
}

void TrajectoryVisual::setDimension(const Ogre::Vector3& dim) {
    for (auto point_visual : trajectoryPointVisualList_) {
        point_visual->setDimension(dim);
    }
}

}  // end of namespace planner_rviz_plugins
