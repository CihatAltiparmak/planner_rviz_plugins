#include <planner_rviz_plugins/trajectory_display.hpp>

namespace planner_rviz_plugins {

TrajectoryDisplay::TrajectoryDisplay() {
    color_property_ = new rviz_common::properties::ColorProperty(
        "Color", QColor(204, 51, 204), "Color to draw the square", this,
        SLOT(updateColorAndAlpha()));

    alpha_property_ = new rviz_common::properties::FloatProperty(
        "Color", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
        SLOT(updateColorAndAlpha()));

    history_length_property_ = new rviz_common::properties::IntProperty(
        "History Length", 1, "Number of prior measurements to display.", this,
        SLOT(updateHistoryLength()));

    history_length_property_->setMin(1);
    history_length_property_->setMax(100000);

    car_dimension_property_ = new rviz_common::properties::VectorProperty(
        "Car Dimension", Ogre::Vector3(3, 3, 3),
        "Dimensions of Car in x, y, z respectly", this,
        SLOT(updateCarDimension()));
}

TrajectoryDisplay::~TrajectoryDisplay() {}

void TrajectoryDisplay::onInitialize() {
    MFDClass::onInitialize();
    updateHistoryLength();
}

void TrajectoryDisplay::reset() {
    MFDClass::reset();
    visuals_.clear();
}

void TrajectoryDisplay::updateColorAndAlpha() {
    float alpha = alpha_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();

    for (size_t i = 0; i < visuals_.size(); i++) {
        visuals_[i]->setColor(color.r, color.g, color.b, alpha);
    }
}

void TrajectoryDisplay::updateHistoryLength() {
    size_t history_length_ =
        static_cast<size_t>(history_length_property_->getInt());
    if (visuals_.size() > history_length_) {
        visuals_.resize(history_length_);
    }
}

void TrajectoryDisplay::updateCarDimension() {
    Ogre::Vector3 car_dim = car_dimension_property_->getVector();

    for (size_t i = 0; i < visuals_.size(); i++) {
        visuals_[i]->setDimension(car_dim);
    }
}

void TrajectoryDisplay::processMessage(
    planner_msgs::msg::Path::ConstSharedPtr msg) {
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if (!context_->getFrameManager()->getTransform(
            msg->header.frame_id, msg->header.stamp, position, orientation)) {
        RCLCPP_INFO(rclcpp::get_logger("trajectory_point_display"),
                    "Error transforming from frame '%s' to frame '%s'",
                    msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
        return;
    }

    // Set the contents of the visual.
    std::shared_ptr<TrajectoryVisual> visual;
    visual.reset(
        new TrajectoryVisual(context_->getSceneManager(), scene_node_));
    visual->setMessage(msg);
    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);

    float alpha = alpha_property_->getFloat();
    Ogre::ColourValue color = color_property_->getOgreColor();
    visual->setColor(color.r, color.g, color.b, alpha);

    Ogre::Vector3 car_dim = car_dimension_property_->getVector();
    visual->setDimension(car_dim);

    size_t history_length_ =
        static_cast<size_t>(history_length_property_->getInt());
    if (visuals_.size() == history_length_) {
        visuals_.pop_back();
    }
    visuals_.push_front(visual);
}

}  // end of namespace planner_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(planner_rviz_plugins::TrajectoryDisplay,
                       rviz_common::Display)