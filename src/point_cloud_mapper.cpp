#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

class PointCloudMapper
{
public:
    PointCloudMapper( ) : nh_("~"), tf_listener_(nh_), accumulation_count_(0)
    {
        // Subscribe to LiDAR point cloud
        point_cloud_sub_ = nh_.subscribe("/livox/lidar", 10, &PointCloudMapper::pointCloudCallback, this);
        
        // Subscribe to ground truth odometry
        odom_sub_ = nh_.subscribe("/ground_truth/state", 10, &PointCloudMapper::odomCallback, this);

        // Initialize the global map
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        global_map_->header.frame_id = "world";

        // Get parameters
        nh_.param<std::string>("output_file", output_file_, "ground_truth_map.pcd");
        nh_.param<int>("downsample_factor", downsample_factor_, 1);
        nh_.param<bool>("save_intermediate", save_intermediate_, false);

        ROS_INFO("Point Cloud Mapper initialized");
        ROS_INFO("Output file: %s", output_file_.c_str());
        ROS_INFO("Downsample factor: %d", downsample_factor_);
    }

    ~PointCloudMapper()
    {
        saveMap();
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        if (!last_odom_)
        {
            ROS_WARN_ONCE("Waiting for ground truth odometry data...");
            return;
        }

        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        pcl::fromROSMsg(*cloud_msg, temp_cloud);

        if (temp_cloud.points.empty())
        {
            return;
        }

        // Apply downsampling if requested
        if (downsample_factor_ > 1)
        {
            pcl::PointCloud<pcl::PointXYZ> downsampled_cloud;
            for (size_t i = 0; i < temp_cloud.points.size(); i += downsample_factor_)
            {
                downsampled_cloud.points.push_back(temp_cloud.points[i]);
            }
            temp_cloud = downsampled_cloud;
        }

        // Transform point cloud from sensor frame to world frame
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        try
        {
            tf::StampedTransform transform;
            tf_listener_.lookupTransform("world", cloud_msg->header.frame_id, cloud_msg->header.stamp, transform);
            pcl_ros::transformPointCloud(temp_cloud, transformed_cloud, transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("Transform failed: %s", ex.what());
            return;
        }

        // Accumulate the transformed cloud
        *global_map_ += transformed_cloud;
        accumulation_count_++;

        if (accumulation_count_ % 10 == 0)
        {
            ROS_INFO("Accumulated %d scans, total points: %ld", accumulation_count_, global_map_->points.size());
        }

        // Optionally save intermediate maps
        if (save_intermediate_ && accumulation_count_ % 50 == 0)
        {
            std::string intermediate_file = "map_intermediate_" + std::to_string(accumulation_count_) + ".pcd";
            pcl::io::savePCDFileASCII(intermediate_file, *global_map_);
            ROS_INFO("Saved intermediate map: %s", intermediate_file.c_str());
        }
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
    {
        last_odom_ = odom_msg;
    }

    void saveMap()
    {
        if (global_map_->points.empty())
        {
            ROS_WARN("Global map is empty, nothing to save");
            return;
        }

        // Save as ASCII PCD
        pcl::io::savePCDFileASCII(output_file_, *global_map_);
        ROS_INFO("Saved map with %ld points to %s", global_map_->points.size(), output_file_.c_str());

        // Also save as binary for faster loading
        std::string binary_file = output_file_.substr(0, output_file_.rfind(".")) + "_binary.pcd";
        pcl::io::savePCDFileBinary(binary_file, *global_map_);
        ROS_INFO("Saved binary map to %s", binary_file.c_str());

        // Save statistics
        saveStatistics();
    }

    void saveStatistics()
    {
        std::string stats_file = output_file_.substr(0, output_file_.rfind(".")) + "_stats.txt";
        std::ofstream stats(stats_file);

        if (!stats.is_open())
        {
            ROS_ERROR("Could not open statistics file");
            return;
        }

        // Calculate statistics
        float min_x = global_map_->points[0].x;
        float max_x = global_map_->points[0].x;
        float min_y = global_map_->points[0].y;
        float max_y = global_map_->points[0].y;
        float min_z = global_map_->points[0].z;
        float max_z = global_map_->points[0].z;

        for (const auto& point : global_map_->points)
        {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }

        stats << "=== Ground Truth Point Cloud Map Statistics ===" << std::endl;
        stats << "Total points: " << global_map_->points.size() << std::endl;
        stats << "Total scans accumulated: " << accumulation_count_ << std::endl;
        stats << "X range: [" << min_x << ", " << max_x << "]" << std::endl;
        stats << "Y range: [" << min_y << ", " << max_y << "]" << std::endl;
        stats << "Z range: [" << min_z << ", " << max_z << "]" << std::endl;
        stats << "Bounding box size: " << (max_x - min_x) << " x " << (max_y - min_y) << " x " << (max_z - min_z) << std::endl;

        stats.close();
        ROS_INFO("Saved statistics to %s", stats_file.c_str());
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber odom_sub_;
    tf::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    nav_msgs::OdometryConstPtr last_odom_;

    int accumulation_count_;
    std::string output_file_;
    int downsample_factor_;
    bool save_intermediate_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_mapper");
    PointCloudMapper mapper;
    ros::spin();
    return 0;
}