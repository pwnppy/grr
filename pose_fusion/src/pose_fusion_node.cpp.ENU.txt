#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <cmath>

class PoseFusionNode : public rclcpp::Node
{
public:
    PoseFusionNode()
        : Node("pose_fusion_node"), is_ref_set_(false)
    {
        // Subscribers for LiDAR and GNSS pose
        lidar_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/lidar_pose_with_covariance", 10,
            std::bind(&PoseFusionNode::lidarPoseCallback, this, std::placeholders::_1));

        gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/gnss_pose_with_covariance", 10,
            std::bind(&PoseFusionNode::gnssPoseCallback, this, std::placeholders::_1));

        // Subscribers for EKF and Filter twist
        ekf_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/ekf_twist", 10,
            std::bind(&PoseFusionNode::ekfTwistCallback, this, std::placeholders::_1));

        filter_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/filter/twist", 10,
            std::bind(&PoseFusionNode::filterTwistCallback, this, std::placeholders::_1));

        // Publisher for final fused pose
        final_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/final/pose_with_covariance", 10);
        fused_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/fused_twist", 10);

        // WGS-84 Parameters
        a_ = 6378137.0;  // Earth's radius in meters
        e2_ = 0.00669437999014;  // Eccentricity squared
    }

private:
    void lidarPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr lidar_msg)
    {
        last_lidar_msg_ = lidar_msg;

        if (last_gnss_msg_)
        {
            fusePoses();
        }
    }

    void gnssPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr gnss_msg)
    {
        last_gnss_msg_ = gnss_msg;

        if (!is_ref_set_)
        {
            ref_lat_ = gnss_msg->pose.pose.position.y;
            ref_lon_ = gnss_msg->pose.pose.position.x;
            ref_alt_ = gnss_msg->pose.pose.position.z;
            is_ref_set_ = true;
        }

        if (last_lidar_msg_)
        {
            fusePoses();
        }
    }

    void ekfTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr ekf_twist_msg)
    {
        last_ekf_twist_msg_ = ekf_twist_msg;

        if (last_filter_twist_msg_)
        {
            fuseTwists();
        }
    }

    void filterTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr filter_twist_msg)
    {
        last_filter_twist_msg_ = filter_twist_msg;

        if (last_ekf_twist_msg_)
        {
            fuseTwists();
        }
    }

    void fusePoses()
    {
        Eigen::Vector3d lidar_pos(last_lidar_msg_->pose.pose.position.x, last_lidar_msg_->pose.pose.position.y, last_lidar_msg_->pose.pose.position.z);
        Eigen::Vector3d gnss_pos = convertGnssToENU(last_gnss_msg_->pose.pose.position);

        geometry_msgs::msg::PoseWithCovarianceStamped fused_pose;
        fused_pose.header.stamp = this->now();
        fused_pose.header.frame_id = "map";

        fused_pose.pose.pose.position.x = lidar_weight_ * lidar_pos.x() + gnss_weight_ * gnss_pos.x();
        fused_pose.pose.pose.position.y = lidar_weight_ * lidar_pos.y() + gnss_weight_ * gnss_pos.y();
        fused_pose.pose.pose.position.z = lidar_weight_ * lidar_pos.z() + gnss_weight_ * gnss_pos.z();

        fused_pose.pose.pose.orientation = last_lidar_msg_->pose.pose.orientation;

        for (size_t i = 0; i < 36; ++i)
        {
            fused_pose.pose.covariance[i] = lidar_weight_ * last_lidar_msg_->pose.covariance[i] +
                                            gnss_weight_ * last_gnss_msg_->pose.covariance[i];
        }

        final_pose_pub_->publish(fused_pose);
    }

    void fuseTwists()
    {
        geometry_msgs::msg::TwistStamped fused_twist;
        fused_twist.header.stamp = this->now();
        fused_twist.header.frame_id = "base_link";  // Adjust frame_id as needed

        fused_twist.twist.linear.x = ekf_twist_weight_ * last_ekf_twist_msg_->twist.linear.x +
                                     filter_twist_weight_ * last_filter_twist_msg_->twist.linear.x;
        fused_twist.twist.linear.y = ekf_twist_weight_ * last_ekf_twist_msg_->twist.linear.y +
                                     filter_twist_weight_ * last_filter_twist_msg_->twist.linear.y;
        fused_twist.twist.linear.z = ekf_twist_weight_ * last_ekf_twist_msg_->twist.linear.z +
                                     filter_twist_weight_ * last_filter_twist_msg_->twist.linear.z;

        fused_twist.twist.angular.x = ekf_twist_weight_ * last_ekf_twist_msg_->twist.angular.x +
                                      filter_twist_weight_ * last_filter_twist_msg_->twist.angular.x;
        fused_twist.twist.angular.y = ekf_twist_weight_ * last_ekf_twist_msg_->twist.angular.y +
                                      filter_twist_weight_ * last_filter_twist_msg_->twist.angular.y;
        fused_twist.twist.angular.z = ekf_twist_weight_ * last_ekf_twist_msg_->twist.angular.z +
                                      filter_twist_weight_ * last_filter_twist_msg_->twist.angular.z;

        fused_twist_pub_->publish(fused_twist);
    }

    Eigen::Vector3d convertGnssToENU(const geometry_msgs::msg::Point& gnss_position)
    {
        double dLat = gnss_position.y - ref_lat_;
        double dLon = gnss_position.x - ref_lon_;
        double dAlt = gnss_position.z - ref_alt_;

        double R_N = a_ / std::sqrt(1 - e2_ * std::sin(ref_lat_ * M_PI / 180.0) * std::sin(ref_lat_ * M_PI / 180.0));
        double R_E = R_N * (1 - e2_);

        double E = R_E * std::cos(ref_lat_ * M_PI / 180.0) * (dLon * M_PI / 180.0);
        double N = R_N * (dLat * M_PI / 180.0);
        double U = dAlt;

        return Eigen::Vector3d(E, N, U);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ekf_twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr filter_twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr final_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr fused_twist_pub_;

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_lidar_msg_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_gnss_msg_;
    geometry_msgs::msg::TwistStamped::SharedPtr last_ekf_twist_msg_;
    geometry_msgs::msg::TwistStamped::SharedPtr last_filter_twist_msg_;

    double lidar_weight_ = 0.5; // Adjust weights as needed
    double gnss_weight_ = 0.5;
    double ekf_twist_weight_ = 0.5;
    double filter_twist_weight_ = 0.5;
    double ref_lat_;
    double ref_lon_;
    double ref_alt_;
    double a_;
    double e2_;
    bool is_ref_set_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseFusionNode>());
    rclcpp::shutdown();
    return 0;
}
