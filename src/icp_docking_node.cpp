#include <icp_docking/icp_docking.h>




using namespace boost::assign;
using namespace boost::adaptors;
using namespace pcl::registration;  

//TODO filter cone-like

inline bool exists_file (const std::string& name) {
    return ( access( name.c_str(), F_OK ) != -1 );
}

void icp_docking_class::LasCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    geometry_msgs::Point32 pnt;
    pnt.z = 0;
    _Lock1.lock();
    _NewPcl.points.clear();
    for (size_t cnt = 0; cnt < msg->ranges.size (); cnt++)
    {
        double ang = msg->angle_min + msg->angle_increment * cnt;
        if((msg->ranges[cnt] < FILT_DIST) && (msg->ranges[cnt] > MIN_DIST) && (ang > _cone_min) && (ang < _cone_max))
        {
            _pnt_filt.x =  (cos(ang) * msg->ranges[cnt]);
            _pnt_filt.y =  (sin(ang) * msg->ranges[cnt]);
        }
        else
        {
            _pnt_filt.x = 0;
            _pnt_filt.y = 0;
            // continue;
        }   

        _NewPcl.points.push_back(_pnt_filt);
    }
   
    _NewPcl.header.frame_id = _Frame_name;
    _NewPcl.header.stamp    = ros::Time::now();

    _pcl_buffer.push_back(_NewPcl);

    _Lock1.unlock();
}

bool icp_docking_class::save_pcl_call(icp_docking::save_pcl::Request  &req, icp_docking::save_pcl::Response &res)
{
    ROS_INFO_STREAM("requesting pcl save index : " <<  req.index);
    const    pcl::PointCloud<pcl::PointXYZ>::Ptr       cloud_save (new pcl::PointCloud<pcl::PointXYZ>);
    int* _Invalid_Data;

    int cnt = 0;
    while(cnt < LASER_FIXED_FILTER_LENGTH)
    {
        sensor_msgs::LaserScanConstPtr LaserFixed = ros::topic::waitForMessage<sensor_msgs::LaserScan>(_laser_scan_name,_nh);
        
        int sizfx  = LaserFixed->ranges.size();
        if(cnt==0)
        {
            cloud_save->width    = sizfx;
            cloud_save->height   = 1;
            cloud_save->is_dense = true;
            cloud_save->points.clear();
            cloud_save->points.resize (cloud_save->width * cloud_save->height);
            _Invalid_Data = new int[sizfx]();
        }
        for (size_t i = 0; i < cloud_save->points.size (); ++i)
        {
            double ang = LaserFixed->angle_min + (LaserFixed->angle_increment * i);
            // ROS_INFO_STREAM(LaserFixed->ranges[i] << " " << ang << std::endl);
            if((LaserFixed->ranges[i] < FILT_DIST) && (LaserFixed->ranges[i] > MIN_DIST) && (ang > _cone_min) && (ang < _cone_max))
            {
                // ROS_INFO_STREAM("in" << std::endl);
                // ROS_INFO_STREAM(ang << std::endl);
                // ROS_INFO_STREAM(LaserFixed->ranges[i] << " " << ang << std::endl  );
                cloud_save->points[i].x = cloud_save->points[i].x + (cos(ang) * LaserFixed->ranges[i]);
                cloud_save->points[i].y = cloud_save->points[i].y + (sin(ang) * LaserFixed->ranges[i]);
                cloud_save->points[i].z = 0.0;
            }
            else
            {
                _Invalid_Data[i] = _Invalid_Data[i] + 1;
            }

        }
        cnt++;
    }

    ROS_INFO_STREAM(" points in ::  " << cloud_save->points.size());

    for (size_t i = 0; i < cloud_save->points.size (); ++i)
    {
        if(_Invalid_Data[i] < 2)//WTF
        {
            cloud_save->points[i].x = cloud_save->points[i].x / (float(LASER_FIXED_FILTER_LENGTH) - float(_Invalid_Data[i]));
            cloud_save->points[i].y = cloud_save->points[i].y / (float(LASER_FIXED_FILTER_LENGTH) - float(_Invalid_Data[i]));
        }
        else
        {
            cloud_save->points[i].x = 0.0;
            cloud_save->points[i].y = 0.0;
        }
        
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for (int i = 0; i < (*cloud_save).size(); i++)
    {
        pcl::PointXYZ pt(cloud_save->points[i].x, cloud_save->points[i].y, cloud_save->points[i].z);
        if ((pt.x==0.0) && (pt.y==0.0))
        {
            inliers->indices.push_back(i);
        }
    }
    extract.setInputCloud(cloud_save);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_save);

    std::string path_save = _path + "/data/fixed_" + std::to_string(req.index) + ".pcd";
    pcl::io::savePCDFileASCII (path_save, *cloud_save);

    _PclFilt.points.clear();
    geometry_msgs::Point32 _pnt_filt;
    for(auto& pnt: cloud_save->points)
    {
        _pnt_filt.x = pnt.x;
        _pnt_filt.y = pnt.y;
        _pnt_filt.z = pnt.z;
        _PclFilt.points.push_back(_pnt_filt);
    }

    _PclFilt.header.frame_id = _Frame_name;
    _PclFilt.header.stamp    = ros::Time::now();
    _filt_pub.publish(_PclFilt);

    res.saved = true;
    return true;
}

bool icp_docking_class::move_pcl_call(icp_docking::move_to_pcl::Request  &req, icp_docking::move_to_pcl::Response &res)
{

    _sub         = _nh.subscribe<sensor_msgs::LaserScan>(_laser_scan_name, 1, &icp_docking_class::LasCallback,this);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr       _cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr             _cloud_in  (new pcl::PointCloud<pcl::PointXYZ>);

    ros::Rate _rt(10);
    std::string path_load = _path + "/data/fixed_" + std::to_string(req.index) + ".pcd";
    if (exists_file(path_load))
    {
        ROS_INFO_STREAM("pcl file found << " << path_load);
        pcl::io::loadPCDFile<pcl::PointXYZ> (path_load, *_cloud_out);
    }
    else
    {
        ROS_ERROR("pcl file not found");
        return false;
    }
    
    _PclFilt.points.clear();
    geometry_msgs::Point32 _pnt_filt;
    for(auto& pnt: _cloud_out->points)
    {
        _pnt_filt.x = pnt.x;
        _pnt_filt.y = pnt.y;
        _pnt_filt.z = pnt.z;
        _PclFilt.points.push_back(_pnt_filt);
    }

    _PclFilt.header.frame_id = _Frame_name;
    _PclFilt.header.stamp    = ros::Time::now();
    _filt_pub.publish(_PclFilt);

    _icp.setMaximumIterations(15);
    
    _icp.setInputSource (_cloud_in);
    _icp.setInputTarget (_cloud_out);
    
    ros::Time start_time = ros::Time::now();

    double outx;
    double outy;
    double outz;
    
    while(ros::ok() && (ros::Time::now() - start_time).toSec() <= _timeout  )
    {
        _Lock1.lock();
        int siz = _NewPcl.points.size();

        _output_pub.publish(_NewPcl);
        
        //PCL for _icp
        _cloud_in->width    = siz;
        _cloud_in->height   = 1;
        _cloud_in->is_dense = true;
        _cloud_in->points.clear();
        _cloud_in->points.resize (_cloud_in->width * _cloud_in->height);
        for (auto it=_pcl_buffer.begin(); it!=_pcl_buffer.end(); ++it)
        {
            for (size_t i = 0; i < _cloud_in->points.size (); ++i)
            {
                _cloud_in->points[i].x = _cloud_in->points[i].x +  it->points[i].x/double(LASER_SCAN_FILTER_LENGTH);
                _cloud_in->points[i].y = _cloud_in->points[i].y +  it->points[i].y/double(LASER_SCAN_FILTER_LENGTH);
                _cloud_in->points[i].z = 0.0;
            }
        }
        

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        for (int i = 0; i < (*_cloud_in).size(); i++)
        {
            pcl::PointXYZ pt(_cloud_in->points[i].x, _cloud_in->points[i].y, _cloud_in->points[i].z);
            if ((pt.x==0.0) && (pt.y==0.0))
            {
                inliers->indices.push_back(i);
            }
        }
        extract.setInputCloud(_cloud_in);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*_cloud_in);

        _icp.align (*_cloud_in);
        _Lock1.unlock();
        _icp_out2 = _icp.getFinalTransformation().cast<double>() ;
        geometry_msgs::WrenchStamped wrench_out;
        wrench_out.header.frame_id = _Frame_name;
        wrench_out.header.stamp    = ros::Time();
        wrench_out.wrench.force.x  = _icp_out2(0,3);
        wrench_out.wrench.force.y  = _icp_out2(1,3);
        wrench_out.wrench.force.z  = _icp_out2(2,3);
        Eigen::Affine3d b;
        b.matrix() = _icp_out2;
        Eigen::Quaterniond qua = Eigen::Quaterniond(b.linear());
        
        Eigen::Vector3d eul = qua.toRotationMatrix().eulerAngles(0, 1, 2);
        wrench_out.wrench.torque.x = eul[0];
        wrench_out.wrench.torque.y = eul[1];
        wrench_out.wrench.torque.z = eul[2];

        _vis_pub.publish( wrench_out );
        // std::cout << "Error : " << _icp_out2(0,3) << " " << _icp_out2(1,3) << " " << eul[2] << std::endl;

        cmd_vel_dock.linear.x  = std::min(std::max(1.0 * _icp_out2(0,3), -0.05), 0.05);
        cmd_vel_dock.linear.y  = std::min(std::max(1.0 * _icp_out2(1,3), -0.05), 0.05);
        cmd_vel_dock.angular.z = std::min(std::max(1.0 *  eul[2]       , -0.05), 0.05);

        outx = _icp_out2(0,3); 
        outy = _icp_out2(1,3); 
        outz = eul[2]; 

        _cmd_vel.publish(cmd_vel_dock);
        
        _rt.sleep();
    }
    _sub.shutdown();
    return true;
}

icp_docking_class::icp_docking_class(ros::NodeHandle* nodehandle):_nh(*nodehandle)
{

    if (!_nh.getParam("laser_scan_topic" , _laser_scan_name )) ROS_ERROR("No fixed laser name param");
    if (!_nh.getParam("frame_name" , _Frame_name )) ROS_ERROR("No fixed laser name param");
    if (!_nh.getParam("timeout" , _timeout )) ROS_ERROR("No timeout name param");
    if (!_nh.getParam("cone_min" , _cone_min )) ROS_ERROR("No cone_min name param");
    if (!_nh.getParam("cone_max" , _cone_max )) ROS_ERROR("No cone_max name param");
    if (!_nh.getParam("cmd_vel_topic" , _cmd_vel_topic )) ROS_ERROR("No cmd_vel_topic name param");
    if (!_nh.getParam("xpid" , _xP )) ROS_ERROR("No xP name param");
    if (!_nh.getParam("ypid" , _yP )) ROS_ERROR("No yP name param");
    if (!_nh.getParam("zpid" , _zP )) ROS_ERROR("No zP name param");

    _save_pcl    = _nh.advertiseService("save_pcl",    &icp_docking_class::save_pcl_call,this);
    _move_to_pcl = _nh.advertiseService("move_to_pcl", &icp_docking_class::move_pcl_call,this);

    _pcl_buffer = boost::circular_buffer<sensor_msgs::PointCloud> (LASER_SCAN_FILTER_LENGTH);




    // _sub         = _nh.subscribe<sensor_msgs::LaserScan>(_laser_scan_name, 1, &icp_docking_class::LasCallback,this);
    _output_pub  = _nh.advertise<sensor_msgs::PointCloud>("pcl_input", 1);
    _filt_pub    = _nh.advertise<sensor_msgs::PointCloud>("pcl_reference", 1,true);
    _vis_pub     = _nh.advertise<geometry_msgs::WrenchStamped>( "wrench", 1 );
    _cmd_vel     = _nh.advertise<geometry_msgs::Twist>(_cmd_vel_topic,1);

}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "icp_docking_node");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    

    icp_docking_class icp_node(&nh);
 
    // nh.setParam("/icp_err/x", outx);
    // nh.setParam("/icp_err/y", outy);
    // nh.setParam("/icp_err/z", outz);

    // nh.setParam("/tasks/SWEEPEE_POPEYE_FROM_ELEVATOR_TO_FUSELAGE_ASSEMBLY/status", "dock_popeye_done");    
    ros::Duration(2).sleep();
    ros::waitForShutdown();
    return 0;
}
