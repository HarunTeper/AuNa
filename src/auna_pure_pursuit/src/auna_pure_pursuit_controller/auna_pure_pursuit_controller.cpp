#include "auna_pure_pursuit/auna_pure_pursuit_controller.hpp"

void AuNaPurePursuitController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros){
    auto node = parent.lock();
    node_ = parent;

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();

    double control_frequency = 20.0;
    node->get_parameter("controller_frequency", control_frequency);
    control_duration_ = 1.0 / control_frequency;
    //print the control duration
    RCLCPP_INFO(logger_, "Control duration: %f", control_duration_);

    //declare parameters
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_velocity", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.5));

    //get parameters
    node->get_parameter(plugin_name_ + ".desired_linear_velocity", desired_linear_velocity_);
    node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
    double transform_tolerance;
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

    global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
    carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
    extended_carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("extended_lookahead_point", 1);
}

void AuNaPurePursuitController::cleanup()
{
    RCLCPP_INFO(
        logger_,
        "Cleaning up controller: %s of type"
        "AuNaPurePursuitController",
        plugin_name_.c_str());
    global_path_pub_.reset();
    carrot_pub_.reset();
    extended_carrot_pub_.reset();
}

void AuNaPurePursuitController::activate()
{
    RCLCPP_INFO(
        logger_,
        "Activating controller: %s of type"
        " AuNaPurePursuitController",
        plugin_name_.c_str());
    global_path_pub_->on_activate();
    carrot_pub_->on_activate();
    extended_carrot_pub_->on_activate();
}

void AuNaPurePursuitController::deactivate()
{
    RCLCPP_INFO(
        logger_,
        "Deactivating controller: %s of type"
        " AuNaPurePursuitController",
        plugin_name_.c_str());
    global_path_pub_->on_deactivate();
    carrot_pub_->on_deactivate();
    extended_carrot_pub_->on_deactivate();
}

geometry_msgs::msg::TwistStamped AuNaPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
    // Transform path to robot base frame
    auto transformed_plan = transformGlobalPlan(pose);

    auto distanceBetweenPoses = [](const auto & p1, const auto & p2) {
        return std::hypot(p1->pose.position.x - p2->pose.position.x, p1->pose.position.y - p2->pose.position.y);
    };

    // Find the first pose which is at a distance greater than the lookahead distance by accumulating the distances over the poses
    auto carrot_pose = transformed_plan.poses.begin();
    double accumulated_distance = 0.0;
    while (carrot_pose != transformed_plan.poses.end()) {
        accumulated_distance += distanceBetweenPoses(carrot_pose, std::next(carrot_pose));
        if (accumulated_distance > lookahead_distance_) {
            break;
        }
        ++carrot_pose;
    }

    // calculate the direct distance between the current pose and the carrot pose
    double direct_distance = distanceBetweenPoses(&pose, carrot_pose);

    auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
    carrot_msg->header = carrot_pose->header;
    carrot_msg->point.x = carrot_pose->pose.position.x;
    carrot_msg->point.y = carrot_pose->pose.position.y;
    carrot_msg->point.z = 0.01;  // publish right over map to stand out
    carrot_pub_->publish(*carrot_msg);

    // From all poses in the path, find the one which is closest to the robot
    auto closest_pose = std::min_element(
        transformed_plan.poses.begin(), transformed_plan.poses.end(),
        [&pose, &distanceBetweenPoses](const auto & p1, const auto & p2) {
            return distanceBetweenPoses(&pose, &p1) < distanceBetweenPoses(&pose, &p2);
    });

    double alpha_max = 1.0;
    double beta_max = 0.2;

    double alpha;
    double beta;

    double length_pv_pw = distanceBetweenPoses(&pose, closest_pose);
    double length_pv_pd = distanceBetweenPoses(&pose, carrot_pose);
    double length_pw_pd = distanceBetweenPoses(closest_pose, carrot_pose);

    // calculate the curvature between the current pose and the carrot pose
    double curv_pd = 2 * sin(carrot_pose->pose.orientation.z - pose.pose.orientation.z) / length_pv_pd;
    double curv_pw = 2 * sin(closest_pose->pose.orientation.z - pose.pose.orientation.z) / length_pv_pw;

    //print the curvatures
    RCLCPP_INFO(logger_, "curv_pd: %f", curv_pd);
    RCLCPP_INFO(logger_, "curv_pw: %f", curv_pw);

    // calculate alpha
    if (length_pv_pw < alpha_max) {
        alpha = length_pv_pw / alpha_max;
    } else {
        alpha = 1.0;
    }

    // calculate beta
    if (curv_pw >= curv_pd) {
        beta = 0.0;
    } else if (curv_pd - curv_pw < beta_max) {
        beta = (curv_pd - curv_pw) / beta_max;
    } else {
        beta = 1.0;
    }

    // calculate tau
    double tau = (1-alpha)*beta;

    //print the alpha, beta and tau
    RCLCPP_INFO(logger_, "alpha: %f", alpha);
    RCLCPP_INFO(logger_, "beta: %f", beta);
    RCLCPP_INFO(logger_, "tau: %f", tau);

    // absolute shift vector length (which is length_pw_pd * tan(the absolute yaw at the closest pose))
    //convert the closest poses orientation to yaw
    double theta = tf2::getYaw(pose.pose.orientation);
    double length_pd_pl = length_pw_pd * std::tan(std::abs(theta));

    // set theta_dl to theta_wd - sign(theta)*pi/2 with theta as the yaw of pose

    double theta_wd = std::atan2(carrot_pose->pose.position.y - closest_pose->pose.position.y, carrot_pose->pose.position.x - closest_pose->pose.position.x);

    double theta_dl = theta_wd - std::copysign(M_PI/2.0, theta);

    // shift the carrot pose by tau * length_pd_pl in the direction of theta_dl
    double x_shift = tau * length_pd_pl * std::cos(theta_dl);
    double y_shift = tau * length_pd_pl * std::sin(theta_dl);

    //print the shift vector
    // RCLCPP_INFO(logger_, "x_shift: %f", x_shift);
    // RCLCPP_INFO(logger_, "y_shift: %f", y_shift);

    //print tau, length_pd_pl and theta_dl
    // RCLCPP_INFO(logger_, "tau: %f", tau);
    // RCLCPP_INFO(logger_, "length_pd_pl: %f", length_pd_pl);
    // RCLCPP_INFO(logger_, "theta_dl: %f", theta_dl);

    // calculate the new carrot pose
    double x_new = carrot_pose->pose.position.x + x_shift;
    double y_new = carrot_pose->pose.position.y + y_shift;

    // create an extended_carrot pose message
    auto extended_carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
    extended_carrot_msg->header = carrot_pose->header;
    extended_carrot_msg->point.x = x_new;
    extended_carrot_msg->point.y = y_new;
    extended_carrot_msg->point.z = 0.01;  // publish right over map to stand out
    extended_carrot_pub_->publish(*extended_carrot_msg);

    // calculate the angle between the closest pose and the extended carrot pose
    double theta_wl = std::atan2(y_new - closest_pose->pose.position.y, x_new - closest_pose->pose.position.x);
    
    // calculate the desired linear velocity by dividing the accumulated distance by the direct distance
    double desired_linear_velocity = desired_linear_velocity_ * (accumulated_distance / direct_distance);
    double desired_angular_velocity = theta_wl - theta_wd;

    // populate and return message
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = pose.header;
    cmd_vel.twist.linear.x = desired_linear_velocity;
    cmd_vel.twist.angular.z = desired_angular_velocity;
    return cmd_vel;
}

void AuNaPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
    global_plan_ = path;
}

void AuNaPurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    desired_linear_velocity_ = base_desired_linear_velocity_;
    } else {
    if (percentage) {
        // Speed limit is expressed in % from maximum speed of robot
        desired_linear_velocity_ = base_desired_linear_velocity_ * speed_limit / 100.0;
    } else {
        // Speed limit is expressed in absolute value
        desired_linear_velocity_ = speed_limit;
    }
    }
}

nav_msgs::msg::Path AuNaPurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
    if (global_plan_.poses.empty()) {
        throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
        throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    const double max_costmap_dim = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
    const double max_transform_dist = max_costmap_dim * costmap->getResolution() / 2.0;

    // First find the closest pose on the path to the robot
    auto transformation_begin = nav2_util::geometry_utils::min_by(
        global_plan_.poses.begin(), global_plan_.poses.end(),
        [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
            return nav2_util::geometry_utils::euclidean_distance(robot_pose, ps);
    });

    // Find points definitely outside of the costmap so we won't transform them.
    auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
        return nav2_util::geometry_utils::euclidean_distance(robot_pose, global_plan_pose) > max_transform_dist;
    });

    // Lambda to transform a PoseStamped from global frame to local
    auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
        geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = robot_pose.header.stamp;
        stamped_pose.pose = global_plan_pose.pose;
        transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
        return transformed_pose;
    };

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(
        transformation_begin, transformation_end,
        std::back_inserter(transformed_plan.poses),
        transformGlobalPoseToLocal);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = robot_pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    global_path_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty()) {
        throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
}

bool AuNaPurePursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

PLUGINLIB_EXPORT_CLASS(AuNaPurePursuitController, nav2_core::Controller)

