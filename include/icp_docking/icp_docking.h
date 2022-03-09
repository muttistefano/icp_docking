#include <ros/ros.h>
#include <math.h> 
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/shared_ptr.hpp>
#include <ros/console.h>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/Point32.h>
#include <icp_docking/save_pcl.h>
#include <icp_docking/move_to_pcl.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/range/adaptor/indexed.hpp>
#include <pcl/io/pcd_io.h>
#include <boost/assign.hpp>
#include <iterator>
#include <eigen3/Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include <boost/circular_buffer.hpp>
#include <iostream>


constexpr int   LASER_FIXED_FILTER_LENGTH  = 10;
constexpr int   LASER_SCAN_FILTER_LENGTH   = 3;
constexpr double FILT_DIST                 = 3.0;
constexpr double MIN_DIST                  = 0.3;

class icp_docking_class
{
    private:

        std::mutex   _Lock1;
        std::string  _laser_scan_name,_Frame_name,_cmd_vel_topic;
        

        double _timeout,_cone_min,_cone_max;
    
        double _xP,_yP,_zP;

        //ROS
        ros::NodeHandle _nh;

        ros::ServiceServer _save_pcl    ;
        ros::ServiceServer _move_to_pcl ;

        ros::Subscriber _sub        ;
        ros::Publisher  _output_pub  ;
        ros::Publisher  _filt_pub    ;
        ros::Publisher  _vis_pub     ;
        ros::Publisher  _cmd_vel     ;

        geometry_msgs::Twist cmd_vel_dock;


        //PCL
        boost::circular_buffer<sensor_msgs::PointCloud>  _pcl_buffer;//

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> _icp;

        sensor_msgs::PointCloud _NewPcl;
        sensor_msgs::PointCloud _PclFilt;
        sensor_msgs::PointCloud _TPcl;
        geometry_msgs::Point32  _pnt_filt;

        sensor_msgs::LaserScanConstPtr _LaserFixed;//
        
        Eigen::Matrix4d _icp_out  = Eigen::Matrix4d::Identity ();
        Eigen::Matrix4d _icp_out2 = Eigen::Matrix4d::Identity ();
        
        std::string _path = ros::package::getPath("icp_docking");

    public:

        explicit icp_docking_class(ros::NodeHandle* nodehandle);

        void LasCallback(const sensor_msgs::LaserScanConstPtr& msg);

        bool save_pcl_call(icp_docking::save_pcl::Request  &req, icp_docking::save_pcl::Response &res);
        bool move_pcl_call(icp_docking::move_to_pcl::Request  &req,icp_docking::move_to_pcl::Response &res);

};