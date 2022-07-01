#include <ros/ros.h>
#include <serial_pkg/serial_commu.h>



int main(int argc, char *argv[])
{
    ros::init(argc,argv,"serial");

    uint64_t count = 0;

    ros::Rate r(1);

    while(ros::ok())
    {
        ROS_INFO("count:%d",count++);
        r.sleep();
    }

    return 0;
}





