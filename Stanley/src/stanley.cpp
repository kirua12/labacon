#include "stanley.h"

Stanley::Stanley(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    pnh.param<double>("L", L, 2.0);
    pnh.param<double>("velocity_gain",velocity_gain,1.0);
    twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //path_sub = nh.subscribe("path", 1, &Stanley::PathPointCallback,this);
    state_sub = nh.subscribe("/state", 1, &Stanley::stateCB, this);
    data_sub= nh.subscribe<sensor_msgs::PointCloud2>("/roi_points",100,&Stanley::PathPointCallback,this);
    ackermann_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);
    boost::thread publish_thread(boost::bind(&Stanley::Publish, this));
}

Stanley::~Stanley()
{

}

void Stanley::PathPointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    path_point = *msg;
    raw_data.clear();
    pcl::fromROSMsg(path_point,raw_data);

}

void Stanley::CurrentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    current_pose = *msg;
    return;
}

double Stanley::steering_cte()
{
    if(raw_data.size()>=2)
    {
        path_pose = raw_data.points[0];
        next_pose = raw_data.points[1];
        float dx = path_pose.x;
        float dy = path_pose.y;

        float yaw = atan2(dy,dx);

        float front_x = L*cos(yaw);
        float front_y = L*sin(yaw);

        min_dist = 1e9;

    //    for(int i = 0;i < path_point.data.size();i++)
    //    {
    //        pcl::PointXYZI tmp_pt;
    //        tmp_pt=path_point.data.at(i);

    //        float x = front_x - tmp_pt.x;
    //        float y = front_y - tmp_pt.y;

    //        float dist = sqrt(pow(x,2) + pow(y,2));

    //        if(dist < min_dist)
    //        {
    //            min_dist = dist;
    //            min_index = i;
    //        }
    //    }
        error_x = path_pose.x - front_x;
        error_y = path_pose.y - front_y;

        cx = next_pose.x - path_pose.x;
        cy = next_pose.y - path_pose.y;

        cyaw = atan2(cy,cx);

        float vec_x = cos(yaw + M_PI/2);
        float vec_y = sin(yaw + M_PI/2);

        error = (vec_x*error_x)+(vec_y*error_y);

        error_cte = atan2(k*error_cte,velocity);

        steering = cyaw + error_cte;
        std::cout<<"steering = "<<steering<<std::endl;

        return steering;
    }
    else
    {
        std::cout<<"fuck you"<<std::endl;
    }
}

void Stanley::stateCB(const std_msgs::String stateMsg)
{
    this->state.data=stateMsg.data;
}


double Stanley::velocity_()
{
    if(this ->state.data == "go")
    {
        velocity = 50*velocity_gain;
        if(velocity>100)
            velocity = 100;
    }
    return velocity;
}

double Stanley::velocity_mini()
{
    if(raw_data.size()>=2)
    {
        velocity = 0.3;
    }
    else
    {
        std::cout<<"응애"<<std::endl;
    }
    return velocity;
}

void Stanley::Publish(void)
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        cmd_vel_.header.stamp = ros::Time::now();
        cmd_vel_.drive.steering_angle = steering_cte();
        cmd_vel_.drive.speed = velocity_mini();
        ackermann_pub.publish(cmd_vel_);

        cmd_vel.linear.x = velocity_();
        cmd_vel.angular.z = steering_cte();
        twist_pub.publish(cmd_vel);
    }
    return;
}
