#include "rclcpp/rclcpp.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/point.hpp>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

class PerceptionNode : public rclcpp::Node
{
    public:
        PerceptionNode()
        : Node("perception_node", rclcpp::NodeOptions()
                                          .allow_undeclared_parameters(true)
                                          .automatically_declare_parameters_from_overrides(true))
        {
            /*
             * SET UP PUBLISHERS
             */
            RCLCPP_INFO(this->get_logger(), "Setting up publishers");

            voxel_grid_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cluster", 1);

            passthrough_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("passthrough_cluster", 1);

            plane_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("plane_cluster", 1);

            euclidean_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("euclidean_cluster", 1);

            stat_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("stat_cluster", 1);

            position_publisher_ =
                this->create_publisher<geometry_msgs::msg::Quaternion>("point_position", 1);

            teste_publisher_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("teste_pointcloud2", 1);

            teste_publisher2_ =
                this->create_publisher<sensor_msgs::msg::PointCloud2>("teste_pointxyzi", 1);


            /*
             * SET UP PARAMETERS
             */
            rclcpp::Parameter cloud_topic_param, world_frame_param, camera_frame_param;

            RCLCPP_INFO(this->get_logger(), "Getting parameters");

            this->get_parameter_or("cloud_topic", cloud_topic_param, rclcpp::Parameter("", "/cloud_fullframe"));
            this->get_parameter_or("world_frame", world_frame_param, rclcpp::Parameter("", "world_frame"));
            this->get_parameter_or("camera_frame", camera_frame_param, rclcpp::Parameter("", "world"));


            cloud_topic = cloud_topic_param.as_string();
            world_frame = world_frame_param.as_string();
            camera_frame = camera_frame_param.as_string();


            /*
             * SET UP SUBSCRIBER
             */
            RCLCPP_INFO(this->get_logger(), "Setting up subscriber");

            cloud_subscriber_ =
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    cloud_topic, 1, std::bind(&PerceptionNode::cloud_callback, this, std::placeholders::_1));

            /*
             * SET UP TF
             */
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

    private:

        /*
         * LISTEN FOR PointCloud2
         */
        void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
        {
            RCLCPP_INFO(this->get_logger(), "Cloud service called; getting a PointCloud2 on topic " + cloud_topic);

            /*
             * TRANSFORM PointCloud2 FROM CAMERA FRAME TO WORLD FRAME
             */

            geometry_msgs::msg::TransformStamped stransform;

            try
            {
                stransform = tf_buffer_->lookupTransform(world_frame, recent_cloud->header.frame_id,
                                                         tf2::TimePointZero, tf2::durationFromSec(3));

            }
            catch (const tf2::TransformException & ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            }

            sensor_msgs::msg::PointCloud2 transformed_cloud;
            pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

            transformed_cloud.fields[3].name = "intensity";
            /* for(int i = 0; i < transformed_cloud.fields.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "PointField array consists of: [%s]", transformed_cloud.fields[i].name.c_str());
                RCLCPP_INFO(this->get_logger(), "TESTE");
            } */

            /*
             * CONVERT PointCloud2 ROS->PCL
             */

            pcl::PointCloud<pcl::PointXYZI> cloud;

/*            RCLCPP_INFO(this->get_logger(), "TESTE pre conversao");
            RCLCPP_INFO(this->get_logger(), "Size of cloud: [%d]", sizeof(cloud[0]));
            RCLCPP_INFO(this->get_logger(), "Size of x field: [%d]", sizeof(cloud[0].x));
            RCLCPP_INFO(this->get_logger(), "Size of y field: [%d]", sizeof(cloud[0].y));
            RCLCPP_INFO(this->get_logger(), "Size of z field: [%d]", sizeof(cloud[0].z));
            RCLCPP_INFO(this->get_logger(), "Size of intensity field: [%d]", sizeof(cloud[0].intensity));
            RCLCPP_INFO(this->get_logger(), "Values: x:%.2f y:%.2f z:%.2f i: %.2f", cloud[0].x, cloud[0].y, cloud[0].z, cloud[0].intensity); */
            teste_publisher_->publish(transformed_cloud);
            
            pcl::fromROSMsg(transformed_cloud, cloud);
/*            RCLCPP_INFO(this->get_logger(), "TESTE pos conversao");
            RCLCPP_INFO(this->get_logger(), "Size of cloud: [%d]", sizeof(cloud[0]));
            RCLCPP_INFO(this->get_logger(), "Size of x field: [%d]", sizeof(cloud[0].x));
            RCLCPP_INFO(this->get_logger(), "Size of y field: [%d]", sizeof(cloud[0].y));
            RCLCPP_INFO(this->get_logger(), "Size of z field: [%d]", sizeof(cloud[0].z));
            RCLCPP_INFO(this->get_logger(), "Size of intensity field: [%d]", sizeof(cloud[0].intensity));
            RCLCPP_INFO(this->get_logger(), "Values: x:%.2f y:%.2f z:%.2f i: %.2f", cloud[0].x, cloud[0].y, cloud[0].z, cloud[0].intensity); */

            this->publishPointCloud(teste_publisher2_, cloud);
            /* RCLCPP_INFO(this->get_logger(), "TESTE2"); */

            /* ========================================
             * Fill Code: VOXEL GRID
             * ========================================*/

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(cloud));
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setInputCloud(cloud_ptr);
            voxel_filter.setLeafSize(float(0.002), float(0.002), float(0.002));
            voxel_filter.filter(*cloud_voxel_filtered);

            /* ========================================
             * Fill Code: PASSTHROUGH FILTER(S)
             * ========================================*/

            pcl::PointCloud<pcl::PointXYZI> xf_cloud, yf_cloud, xyzi_filtered_cloud;
            pcl::PassThrough<pcl::PointXYZI> pass_x;
            pass_x.setInputCloud(cloud_voxel_filtered);
            pass_x.setFilterFieldName("x");
            pass_x.setFilterLimits(0.5, 5.0);
            pass_x.filter(xf_cloud);

            pcl::PointCloud<pcl::PointXYZI>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(xf_cloud));
            pcl::PassThrough<pcl::PointXYZI> pass_y;
            pass_y.setInputCloud(xf_cloud_ptr);
            pass_y.setFilterFieldName("y");
            pass_y.setFilterLimits(-2.0, 2.0);
            pass_y.filter(yf_cloud);

            pcl::PointCloud<pcl::PointXYZI>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(yf_cloud));
            pcl::PassThrough<pcl::PointXYZI> pass_z;
            pass_z.setInputCloud(yf_cloud_ptr);
            pass_z.setFilterFieldName("z");
            pass_z.setFilterLimits(-2.0, 0.0);
            pass_z.filter(xyzi_filtered_cloud);

            /* ========================================
             * Fill Code: CROPBOX (Optional)
             * ========================================*/


            /* ========================================
             * Fill Code: PLANE SEGEMENTATION
             * ========================================*/

            pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZI>(xyzi_filtered_cloud));
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZI> ());

            pcl::SACSegmentation<pcl::PointXYZI> seg;
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setMaxIterations (200);
            seg.setDistanceThreshold (0.040);

            seg.setInputCloud (cropped_cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.") ;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZI> extract;
            extract.setInputCloud (cropped_cloud);
            extract.setIndices(inliers);
            extract.setNegative (false);

            // Get the points associated with the planar surface
            extract.filter (*cloud_plane);
            RCLCPP_INFO(this->get_logger(),
                        "PointCloud2 representing the planar component: '%lu' data points.", cloud_plane->points.size());

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);


            /* ========================================
             * Fill Code: EUCLIDEAN CLUSTER EXTRACTION
             * ========================================*/

            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
            *cloud_filtered = *cloud_f;
            tree->setInputCloud (cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance (0.050);
            ec.setMinClusterSize (1);
            ec.setMaxClusterSize (10000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);

            std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> pc2_clusters;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > clusters;

            int j = 0;
            for (const auto& cluster : cluster_indices)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);

                for (const auto& idx : cluster.indices) {
                    cloud_cluster->points.push_back((*cloud_filtered)[idx]);
                }

                cloud_cluster->width = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true; 
                RCLCPP_INFO(this->get_logger(), "Cluster has '%lu' points", cloud_cluster->points.size());
                clusters.push_back(cloud_cluster);
                sensor_msgs::msg::PointCloud2::SharedPtr tempROSMsg(new sensor_msgs::msg::PointCloud2);
                pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
                pc2_clusters.push_back(tempROSMsg);

                j++;

            }
            RCLCPP_INFO(this->get_logger(), "Largest cluster has '%lu' points", clusters.at(0)->points.size());

            /* ========================================
             * Fill Code: STATISTICAL OUTLIER REMOVAL
             * ========================================*/
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud_ptr= clusters.at(0);
            pcl::PointCloud<pcl::PointXYZI>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;

            sor.setInputCloud (cluster_cloud_ptr);
            sor.setMeanK (50);
            sor.setStddevMulThresh (1.0);
            sor.setMeanK (50);
            sor.setStddevMulThresh (1.0);
            sor.filter (*sor_cloud_filtered);


            /* ========================================
             * Fill Code: PUBLISH OTHER MARKERS (OPTIONAL)
             * ========================================*/


            /* ========================================
             * BROADCAST TRANSFORM (LiDAR to Obstacle)
             * ========================================*/
            
            std::unique_ptr<tf2_ros::TransformBroadcaster> br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            geometry_msgs::msg::TransformStamped part_transform;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            part_transform.transform.rotation.x = q.x();
            part_transform.transform.rotation.y = q.y();
            part_transform.transform.rotation.z = q.z();
            part_transform.transform.rotation.w = q.w();

            //Here x,y, and z should be calculated based on the PointCloud2 filtering results
            part_transform.transform.translation.x = sor_cloud_filtered->at(0).x;
            part_transform.transform.translation.y = sor_cloud_filtered->at(0).y;
            part_transform.transform.translation.z = sor_cloud_filtered->at(0).z;
            part_transform.header.stamp = this->get_clock()->now();
            part_transform.header.frame_id = world_frame;
            part_transform.child_frame_id = "obstacle";

            //Publish the position of the filtered Points from PointCloud2
            geometry_msgs::msg::Quaternion point_msg;

            point_msg.x = part_transform.transform.translation.x;
            point_msg.y = part_transform.transform.translation.y;
            point_msg.z = part_transform.transform.translation.z;

            br->sendTransform(part_transform);

            /* ========================================
             * BROADCAST TRANSFORM (Map to LiDAR)
             * ========================================*/

            /* // Assume that the transformation between 'world' and 'map' is known
            geometry_msgs::msg::TransformStamped world_to_map_transform;
            world_to_map_transform.header.stamp = this->get_clock()->now();
            world_to_map_transform.header.frame_id = "map";  // target frame
            world_to_map_transform.child_frame_id = world_frame;  // source frame

            // Set the translation and rotation based on known values (replace with your SLAM logic)
            world_to_map_transform.transform.translation.x = 1.0;  // Replace with SLAM-generated translation
            world_to_map_transform.transform.translation.y = 2.0;  // Replace with SLAM-generated translation
            world_to_map_transform.transform.translation.z = 0.0;
            world_to_map_transform.transform.rotation.x = 0.0;
            world_to_map_transform.transform.rotation.y = 0.0;
            world_to_map_transform.transform.rotation.z = 0.0;
            world_to_map_transform.transform.rotation.w = 1.0;

            br->sendTransform(world_to_map_transform); */


            /* ========================================
             * Fill Code: POLYGONAL SEGMENTATION
             * ========================================*/


            /* ========================================
             * CONVERT PointCloud2 PCL->ROS
             * PUBLISH CLOUD
             * Fill Code: UPDATE AS NECESSARY
             * ========================================*/

            this->publishPointCloud(voxel_grid_publisher_, *cloud_voxel_filtered);
            this->publishPointCloud(passthrough_publisher_, xyzi_filtered_cloud);
            this->publishPointCloud(plane_publisher_, *cloud_f);
            this->publishPointCloud(euclidean_publisher_, *(clusters.at(0)));
            this->publishPointCloud(stat_publisher_, *sor_cloud_filtered);
            position_publisher_->publish(point_msg);
            teste_publisher_->publish(transformed_cloud);
            this->publishPointCloud(teste_publisher2_, cloud); 

            // Use smart pointer to deallocate memory
            /* cloud_ptr.reset();
            cloud_voxel_filtered.reset();
            xf_cloud_ptr.reset();
            yf_cloud_ptr.reset();
            cropped_cloud.reset();
            cloud_f.reset();
            cloud_filtered.reset();
            cloud_plane.reset(); */

        }

        void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                               pcl::PointCloud<pcl::PointXYZI> point_cloud) 
        {

            sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);
            pcl::toROSMsg(point_cloud, *pc2_cloud);
            pc2_cloud->header.frame_id = world_frame;
            pc2_cloud->header.stamp = this->get_clock()->now();
            publisher->publish(*pc2_cloud);
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;

        /*
         * Publishers
         */
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_grid_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passthrough_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr euclidean_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stat_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr position_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr teste_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr teste_publisher2_;


        /*
         * Parameters
         */
        std::string cloud_topic;
        std::string world_frame;
        std::string camera_frame;
        std::string obstacle;
        std::string map;

        /*
         * TF
         */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> br;

};


int main(int argc, char *argv[])
{
    /*
     * INITIALIZE ROS NODE
     */
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
