#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan scan;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_)
{
    float nandeyanen = scan_->range_max;
    scan.header = scan_->header;
    scan.angle_min = scan_->angle_min;
    scan.angle_max = scan_->angle_max;
    scan.angle_increment = scan_->angle_increment;
    scan.time_increment = scan_->time_increment;
    scan.scan_time = scan_->scan_time;
    scan.range_min = scan_->range_min;
    scan.range_max = scan_->range_max;
    scan.ranges = scan_->ranges;
    scan.intensities = scan_->intensities;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "karugamo");
    ros::NodeHandle nh;
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_pub", 10);
    ros::Rate loop_rate(10);

    ros::spinOnce();
    int count = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        // // ロボットの正面のセンサの値の表示
        if (count == 5)
        {
            int angle_center = scan.ranges.size() / 2;
            std::cout << "scan.ranges[" << angle_center << "] : " << scan.ranges[384] << std::endl;
        }
        count++;

        // // rvizへとscanの値をそのままPublish
        scan_pub.publish(scan);

        loop_rate.sleep();
    }
}