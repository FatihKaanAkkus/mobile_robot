#include <cmath>
#include <tf/transform_datatypes.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/urdfdom_compatibility.h>
#include <boost/assign.hpp>

#include <my_controller_pkg/MobileController.h>

namespace my_controller_pkg
{
    MobileController::MobileController():
        open_loop_(false),
        wheel_separation_(0.0),
        wheel_radius_(0.0),
        cmd_vel_timeout_(0.5),
        base_frame_id_("base_link"),
        odom_frame_id_("odom"),
        enable_odom_tf_(true),
        wheel_joints_size_(0),
        publish_cmd_(false)
    {}

    MobileController::~MobileController()
    {
        sub_cmd_.shutdown();
    }

    bool MobileController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh)
    {
        // Parse for complete namespace
        const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        name_ = complete_ns.substr(id + 1);

        // Wheels
        std::vector<std::string> left_wheel_names, right_wheel_names;

        if(!controller_nh.getParam("left_wheel", left_wheel_names))
        {
            ROS_ERROR_STREAM_NAMED(name_, "'left_wheel' param is needed.");
            return false;
        }

        if(left_wheel_names.size() == 0)
        {
            ROS_ERROR_STREAM_NAMED(name_, "'left_wheel' param is an empty list.");
            return false;
        }

        if(!controller_nh.getParam("right_wheel", right_wheel_names))
        {
            ROS_ERROR_STREAM_NAMED(name_, "'right_wheel' param is needed.");
            return false;
        }

        if(right_wheel_names.size() == 0)
        {
            ROS_ERROR_STREAM_NAMED(name_, "'right_wheel' param is an empty list.");
            return false;
        }

        if(left_wheel_names.size() != right_wheel_names.size())
        {
            ROS_ERROR_STREAM_NAMED(name_,
                "# left wheels (" << left_wheel_names.size() << ") is not equal to " << 
                "# right wheels (" << right_wheel_names.size() << ").");
            return false;
        }
        else 
        {
            wheel_joints_size_ = left_wheel_names.size();
            
            left_wheel_joints_.resize(wheel_joints_size_);
            right_wheel_joints_.resize(wheel_joints_size_);
        }

        // Odometry
        controller_nh.param("publish_rate", publish_rate_, 50.0);
        ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at " << publish_rate_ << "Hz.");
        publish_period_ = ros::Duration(1.0 / publish_rate_);
        
        controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
        ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

        controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
        ROS_INFO_STREAM_NAMED(name_, "Odometry frame_id set to " << odom_frame_id_);

        controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
        ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_ ? "enabled":"disabled"));

        // Velocity and acceleration limits
        controller_nh.param("linear/x/has_velocity_limits"      , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
        controller_nh.param("linear/x/has_acceleration_limits"  , limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
        controller_nh.param("linear/x/has_jerk_limits"          , limiter_lin_.has_jerk_limits        , limiter_lin_.has_jerk_limits        );
        controller_nh.param("linear/x/max_velocity"             , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
        controller_nh.param("linear/x/min_velocity"             , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
        controller_nh.param("linear/x/max_acceleration"         , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
        controller_nh.param("linear/x/min_acceleration"         , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );
        controller_nh.param("linear/x/max_jerk"                 , limiter_lin_.max_jerk               ,  limiter_lin_.max_jerk              );
        controller_nh.param("linear/x/min_jerk"                 , limiter_lin_.min_jerk               , -limiter_lin_.max_jerk              );

        controller_nh.param("angular/z/has_velocity_limits"     , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
        controller_nh.param("angular/z/has_acceleration_limits" , limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
        controller_nh.param("angular/z/has_jerk_limits"         , limiter_ang_.has_jerk_limits        , limiter_ang_.has_jerk_limits        );
        controller_nh.param("angular/z/max_velocity"            , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
        controller_nh.param("angular/z/min_velocity"            , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
        controller_nh.param("angular/z/max_acceleration"        , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
        controller_nh.param("angular/z/min_acceleration"        , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );
        controller_nh.param("angular/z/max_jerk"                , limiter_ang_.max_jerk               ,  limiter_ang_.max_jerk              );
        controller_nh.param("angular/z/min_jerk"                , limiter_ang_.min_jerk               , -limiter_ang_.max_jerk              );

        // Publish limited velocity
        controller_nh.param("publish_cmd", publish_cmd_, publish_cmd_);

        // Parameter
        if(!controller_nh.getParam("wheel_separation", wheel_separation_))
        {
            ROS_ERROR_STREAM_NAMED(name_, "'wheel_separation' parameter is needed.");
            return false;
        }

        // Parameter
        if(!controller_nh.getParam("wheel_radius", wheel_radius_))
        {
            ROS_ERROR_STREAM_NAMED(name_, "'wheel_radius' parameter is needed.");
            return false;
        }

        // Setup odometry parameters
        odometry_.setWheelParams(wheel_separation_, wheel_radius_, wheel_radius_);
        ROS_INFO_STREAM_NAMED(name_,
            "Odometry params: wheel separation: " << wheel_separation_
            << ", left wheel radius: "  << wheel_radius_
            << ", right wheel radius: " << wheel_radius_);

        setOdomPubFields(root_nh, controller_nh);

        int velocity_rolling_window_size = 10;
        controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
        ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of "
            << velocity_rolling_window_size << ".");

        odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

        // Parameter
        if(!controller_nh.getParam("cmd_vel_timeout", cmd_vel_timeout_))
        {
            ROS_ERROR_STREAM_NAMED(name_, "'cmd_vel_timeout' parameter is needed.");
            return false;
        }

        // Limited velocity publisher
        if(publish_cmd_)
        {
            cmd_vel_pub_ = controller_nh.advertise<geometry_msgs::TwistStamped>("cmd_vel_out", (uint32_t) 100);
        }

        // Joint handles
        for(size_t i=0; i<wheel_joints_size_; ++i)
        {
            ROS_INFO_STREAM_NAMED(name_,
                "Adding left wheel with joint name: " << left_wheel_names[i] <<
                " and right wheel with joint name: " << right_wheel_names[i]);
            left_wheel_joints_[i] = hw->getHandle(left_wheel_names[i]);
            right_wheel_joints_[i] = hw->getHandle(right_wheel_names[i]);
        }

        // Subscribe to command topic
        sub_cmd_ = controller_nh.subscribe("cmd_vel", 1, &MobileController::cmd_vel_Callback, this);

        return true;
    }

    void MobileController::starting(const ros::Time& time)
    {
        brake();
        last_state_publish_time_ = time;
        odometry_.init(time);
    }

    void MobileController::update(const ros::Time& time, const ros::Duration& period)
    {
        if(open_loop_)
        {
            odometry_.updateOpenLoop(last0_cmd_.lin, last0_cmd_.ang, time);
        }
        else 
        {
            double left_pos = 0.0;
            double right_pos = 0.0;

            for(size_t i=0; i<wheel_joints_size_; ++i)
            {
                const double lp = left_wheel_joints_[i].getPosition();
                const double rp = right_wheel_joints_[i].getPosition();
                
                if(std::isnan(lp) || std::isnan(rp))
                    return;
                
                left_pos += lp;
                right_pos += rp;
            }

            left_pos /= wheel_joints_size_;
            right_pos /= wheel_joints_size_;

            odometry_.update(left_pos, right_pos, time);
        }

        if(time - last_state_publish_time_ > publish_period_)
        {
            last_state_publish_time_ = time;

            // Compute orientation
            const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

            // Odom message
            if(odom_pub_->trylock())
            {
                odom_pub_->msg_.header.stamp = time;
                odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
                odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
                odom_pub_->msg_.pose.pose.orientation = orientation;
                odom_pub_->msg_.twist.twist.linear.x = odometry_.getLinear();
                odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
                odom_pub_->unlockAndPublish();
            }

            // Tf /odom frame
            if(enable_odom_tf_ && tf_odom_pub_->trylock())
            {
                geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
                odom_frame.header.stamp = time;
                odom_frame.transform.translation.x = odometry_.getX();
                odom_frame.transform.translation.y = odometry_.getY();
                odom_frame.transform.rotation = orientation;
                tf_odom_pub_->unlockAndPublish();
            }
        }

        // Movement
        Commands curr_cmd = command_;
        const double dt = (time - curr_cmd.stamp).toSec();

        // Check if cmd_vel timed out
        if(dt > cmd_vel_timeout_)
        {
            curr_cmd.lin = 0.0;
            curr_cmd.ang = 0.0;
        }

        // Limiter
        const double cmd_dt(period.toSec());

        limiter_lin_.limit(curr_cmd.lin, last0_cmd_.lin, last1_cmd_.lin, cmd_dt);
        limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

        last1_cmd_ = last0_cmd_;
        last0_cmd_ = curr_cmd;

        // Publisher for executed and limited command
        if(publish_cmd_)
        {
            cmd_vel_pub_msg.header.stamp = time;
            cmd_vel_pub_msg.twist.linear.x = curr_cmd.lin;
            cmd_vel_pub_msg.twist.angular.z = curr_cmd.ang;
            cmd_vel_pub_.publish(cmd_vel_pub_msg);
        }

        // Wheel velocities
        const double vel_left = (curr_cmd.lin - curr_cmd.ang * wheel_separation_ / 2.0) / wheel_radius_;
        const double vel_right = (curr_cmd.lin + curr_cmd.ang * wheel_separation_ / 2.0) / wheel_radius_;

        // Set velocities
        for(size_t i=0; i<wheel_joints_size_; ++i)
        {
            left_wheel_joints_[i].setCommand(vel_left);
            right_wheel_joints_[i].setCommand(vel_right);
        }

        ROS_DEBUG_STREAM_NAMED(name_, "vel_left: " << vel_left << ", vel_right: " << vel_right);
    }

    void MobileController::stopping(const ros::Time& time)
    {
        brake();
    }

    void MobileController::brake()
    {
        const double vel = 0.0;
        for (size_t i=0; i<wheel_joints_size_; ++i)
        {
            left_wheel_joints_[i].setCommand(vel);
            right_wheel_joints_[i].setCommand(vel);
        }
    }

    void MobileController::cmd_vel_Callback(const geometry_msgs::Twist& msg)
    {
        if(isRunning())
        {
            command_struct_.lin = (double) msg.linear.x;
            command_struct_.ang= (double) msg.angular.z;
            command_struct_.stamp = ros::Time::now();

            command_ = command_struct_;

            ROS_DEBUG_STREAM_NAMED(name_,
                "Added values to command. "
                << "Lin: "   << command_.lin << ", "
                << "Ang: "   << command_.ang << ", "
                << "Stamp: " << command_.stamp);
        }
        else 
        {
            ROS_ERROR_STREAM_NAMED(name_, "Can't accept new commands. Controller is not running.");
        }
    }
    
    void MobileController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        // Get and check params for covariances
        XmlRpc::XmlRpcValue pose_cov_list;
        controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
        ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(pose_cov_list.size() == 6);
        for (int i = 0; i < pose_cov_list.size(); ++i)
        ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        XmlRpc::XmlRpcValue twist_cov_list;
        controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
        ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(twist_cov_list.size() == 6);
        for (int i = 0; i < twist_cov_list.size(); ++i)
        ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        // Setup odometry realtime publisher + odom message constant fields
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
        odom_pub_->msg_.header.frame_id = odom_frame_id_;
        odom_pub_->msg_.child_frame_id = base_frame_id_;
        odom_pub_->msg_.pose.pose.position.z = 0;
        odom_pub_->msg_.pose.covariance = boost::assign::list_of
            (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
            (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
            (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
            (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
            (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
            (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
        odom_pub_->msg_.twist.twist.linear.y  = 0;
        odom_pub_->msg_.twist.twist.linear.z  = 0;
        odom_pub_->msg_.twist.twist.angular.x = 0;
        odom_pub_->msg_.twist.twist.angular.y = 0;
        odom_pub_->msg_.twist.covariance = boost::assign::list_of
            (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
            (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
            (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
            (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
            (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
            (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
        tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
        tf_odom_pub_->msg_.transforms.resize(1);
        tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
        tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
        tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
    }

}

PLUGINLIB_EXPORT_CLASS(my_controller_pkg::MobileController, controller_interface::ControllerBase)
