#include <planner_rviz_plugins/trajectory_point_visual.hpp>

namespace planner_rviz_plugins {

TrajectoryPointVisual::TrajectoryPointVisual(Ogre::SceneManager* scene_manager,
                                             Ogre::SceneNode* parent_node) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();

    pose_shape_.reset(new rviz_rendering::Shape(rviz_rendering::Shape::Cube,
                                                scene_manager_, frame_node_));
}

TrajectoryPointVisual::~TrajectoryPointVisual() {
    scene_manager_->destroySceneNode(frame_node_);
}

void TrajectoryPointVisual::setMessage(
    planner_msgs::msg::Point::ConstSharedPtr msg) {
    Ogre::Vector3 pos_vec(msg->x, msg->y, 0.0);

    tf2::Quaternion q;
    // Create a quaternion from roll/pitch/yaw in radians (0, 0, 0)
    q.setRPY(msg->yaw, 0.0, 0.0);
    Ogre::Quaternion rot_qua(q.x(), q.y(), q.z(), q.w());

    // float length = pos_vec.length();
    // Ogre::Vector3 scale(length, length, 0.1);
    // pose_shape_->setScale(scale);
    pose_shape_->setPosition(pos_vec);
    pose_shape_->setOrientation(rot_qua);
    // pose_shape_->setDirection(acc);
}

void TrajectoryPointVisual::setFramePosition(const Ogre::Vector3& position) {
    frame_node_->setPosition(position);
}

void TrajectoryPointVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
    frame_node_->setOrientation(orientation);
}

void TrajectoryPointVisual::setColor(float r, float g, float b, float a) {
    pose_shape_->setColor(r, g, b, a);
}

void TrajectoryPointVisual::setDimension(const Ogre::Vector3& dim) {
    pose_shape_->setScale(dim);
}

}  // end of namespace planner_rviz_plugins
