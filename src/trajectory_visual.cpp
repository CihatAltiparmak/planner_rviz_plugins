#include <planner_rviz_plugins/trajectory_visual.hpp>

namespace planner_rviz_plugins {

TrajectoryVisual::TrajectoryVisual(Ogre::SceneManager* scene_manager,
                                   Ogre::SceneNode* parent_node) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();

    pose_shape_.reset(new rviz_rendering::Shape(rviz_rendering::Shape::Cube,
                                                scene_manager_, frame_node_));
}

TrajectoryVisual::~TrajectoryVisual() {
    scene_manager_->destroySceneNode(frame_node_);
}

void TrajectoryVisual::setMessage(
    geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
    auto pos = msg->pose.position;
    Ogre::Vector3 acc(static_cast<float>(pos.x), static_cast<float>(pos.y),
                      static_cast<float>(pos.z));
    float length = acc.length();

    Ogre::Vector3 scale(length, length, length);
    pose_shape_->setScale(scale);
    // pose_shape_->setDirection(acc);
}

void TrajectoryVisual::setFramePosition(const Ogre::Vector3& position) {
    frame_node_->setPosition(position);
}

void TrajectoryVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
    frame_node_->setOrientation(orientation);
}

void TrajectoryVisual::setColor(float r, float g, float b, float a) {
    pose_shape_->setColor(r, g, b, a);
}

}  // end of namespace planner_rviz_plugins
