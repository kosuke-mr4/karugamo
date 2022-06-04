#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// add for morving
#include "std_msgs/Int64.h"
#include "geometry_msgs/Twist.h"

// odom
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

#include "math.h"

sensor_msgs::LaserScan scan;

nav_msgs::Odometry pos;
geometry_msgs::Twist pub_msg;

double roll, pitch, yaw;
float x, y, z;

// はじめに取得した値を格納
double scan_coord[726][2];

double pole1[100][2] = {};
double pole2[100][2] = {};

double pole1_cecnter[2] = {};
double pole2_cecnter[2] = {};

double x_max_coord[2] = {-100, -100};
double y_max_coord[2] = {-100, -100};

double center_of_poles[2] = {0, 0};

int default_rightindex = 230;
int default_leftindex = 500;

int min_leg_index = 1000;
int max_leg_index = 0;

int diff_index = 30;

int isRecognized = 0;

// Urgの値を二次元空間で保存
void scan2coord()
{
    int i = 0;
    while (i < scan.ranges.size())
    {
        double th = i * 255.0 / 725.0;
        th = th - 135.0; // zikki
        // th = th - 120.0;
        double theta = th * M_PI / 180.0;
        if (!(scan.ranges[i] <= 1.5) || scan.ranges[i] < 0.1) // 1m以上は無視する
            scan.ranges[i] = 100.0;
        scan_coord[i][0] = scan.ranges[i] * cos(theta);
        scan_coord[i][1] = scan.ranges[i] * sin(theta);
        i++;
    }
}

double returnDistance(double x, double y)
{
    return (sqrt(pow(x, 2.0) + pow(y, 2.0)));
}

void setObject(int rightindex, int leftindex)
{

    pole1_cecnter[0] = 0;
    pole1_cecnter[1] = 0;
    pole2_cecnter[0] = 0;
    pole2_cecnter[1] = 0;

    //指定範囲で足の認識を行う
    while (isRecognized == 0)
    {
        // std::cout << "checking..." << std::endl;

        scan2coord();

        for (int i = rightindex; i < leftindex; i++)
        {
            // std::cout << "x :" << scan_coord[i][0];
            // std::cout << "y :" << scan_coord[i][1] << std::endl;

            if (abs(scan_coord[i][0]) < 1.5 && abs(scan_coord[i][1]) < 1.5) // isfinite(scan_coord[i][0])
            {
                // std::cout << "x :" << scan_coord[i][0];
                // std::cout << ", y :" << scan_coord[i][1] << std::endl;
                // x
                if (scan_coord[i][0] > x_max_coord[0])
                {
                    x_max_coord[0] = scan_coord[i][0];
                    x_max_coord[1] = scan_coord[i][1];
                }

                // y
                // hugou ga guimon
                // if (abs(scan_coord[i][1]) > abs(y_max_coord[1]))
                if (scan_coord[i][1] > y_max_coord[1])
                {
                    y_max_coord[0] = scan_coord[i][0];
                    y_max_coord[1] = scan_coord[i][1];
                }

                // min , max の更新
                if (i < min_leg_index)
                {
                    min_leg_index = i;
                }

                if (max_leg_index < i)
                {
                    max_leg_index = i;
                }
            }
            else
            // Nan no taiou
            {
                if (!(x_max_coord[0] == -100)) // 値がなにか残ってたらコミットしてクリア
                {
                    if (pole1_cecnter[0] == 0)
                    {
                        pole1_cecnter[0] = (x_max_coord[0] + y_max_coord[0]) / 2;
                        pole1_cecnter[1] = (x_max_coord[1] + y_max_coord[1]) / 2;

                        std::cout << " pole1 is changed : " << pole1_cecnter[0] << pole1_cecnter[1] << std::endl;

                        x_max_coord[0] = -100;
                        x_max_coord[1] = -100;

                        y_max_coord[0] = -100;
                        y_max_coord[1] = -100;
                    }
                    else if (pole2_cecnter[0] == 0)
                    {
                        pole2_cecnter[0] = (x_max_coord[0] + y_max_coord[0]) / 2;
                        pole2_cecnter[1] = (x_max_coord[1] + y_max_coord[1]) / 2;

                        std::cout << " pole2 is changed : " << pole2_cecnter[0] << pole2_cecnter[1] << std::endl;

                        x_max_coord[0] = -100;
                        x_max_coord[1] = -100;

                        y_max_coord[0] = -100;
                        y_max_coord[1] = -100;
                    }
                }
            }
        }
        double center1Range = returnDistance(pole1_cecnter[0], pole2_cecnter[1]);
        double center2Range = returnDistance(pole2_cecnter[0], pole2_cecnter[1]);

        if (0.2 < center1Range && center1Range < 1.5 && 0.2 < center2Range && center2Range < 1.5 && isRecognized == 0)
        {
            isRecognized = 1;
        }
        else
        {

            std::cout << "x_max_coord :  x : " << x_max_coord[0] << ", y : " << x_max_coord[1] << std::endl;
            std::cout << "y_max_coord :  x : " << y_max_coord[0] << ", y : " << y_max_coord[1] << std::endl;

            std::cout << "center 1 , x :" << pole1_cecnter[0];
            std::cout << " y : " << pole1_cecnter[1] << std::endl;

            std::cout << "center 2 , x :" << pole2_cecnter[0];
            std::cout << " y : " << pole2_cecnter[1] << std::endl;
            std::cout << "invalid pole range" << std::endl;

            isRecognized = 1;
        }
    }
}

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

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos.header = msg->header;
    pos.child_frame_id = msg->child_frame_id;
    pos.pose = msg->pose;
    pos.twist = msg->twist;

    // (x, y, z)の取得
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    // roll, pitch, yawへの変換、取得
    tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    m.getRPY(roll, pitch, yaw);
}

void moveFormard(ros::Publisher pub, float x)
{
    pub_msg.angular.z = 0;
    pub_msg.linear.x = x; // heisinn sokudo
    pub.publish(pub_msg);
}

void moveStop(ros::Publisher pub)
{
    pub_msg.angular.z = 0;
    pub_msg.linear.x = 0;
    pub.publish(pub_msg);
}

void moveCtrl(ros::Publisher pub, float x, float z)
{

    pub_msg.angular.z = z;
    pub_msg.linear.x = x;
    pub.publish(pub_msg);
}

// (x, y)に向かうプログラム
void go_pos_GL(double x, double y)
{
    // 制御のパラメータ
    const double k_v = 0.8;
    const double k_w = 3.0;

    //最大速度、最大角速度
    const double v_max = 0.3;
    const double w_max = 0.6;

    tf::Quaternion quat(pos.pose.pose.orientation.x, pos.pose.pose.orientation.y, pos.pose.pose.orientation.z, pos.pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 現在のロボットの位置、姿勢
    double x0 = pos.pose.pose.position.x;
    double y0 = pos.pose.pose.position.y;
    double theta = yaw;

    // 目標の角度
    double th = atan2(y, x);

    // 目標点がロボットの正面にあるか背後にあるのかを確認する値
    double sign = 0;
    // 目標点との角度
    double phai = th - theta;

    // 角度を-πからπに収める
    while ((phai <= -M_PI) || (phai > M_PI / 2))
    {
        if (phai <= -M_PI)
        {
            phai += M_PI;
        }
        else
        {
            phai -= M_PI;
        }
    }
    if ((phai < -(M_PI / 2)) || (phai >= (M_PI / 2)))
    {
        sign = -1;
    }
    else
    {
        sign = 1;
    }

    // ロボットと目標点との距離
    double distance = sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0));

    // 目標点の座標の方向とロボットの向きとの差が大きいほど値が小さくなる角速度を設定
    double w = k_w * phai;

    // 最大角速度より大きければ最大角速度に設定
    if (w > w_max)
        w = w_max;
    else if (w < -w_max)
        w = -w_max;
    // 目標点の座標に近いほど値が小さくなる速度を設定
    double v = k_w * distance * sign;

    // 最大速度より大きければ最大速度に設定
    if (v > v_max)
        v = v_max;
    else if (v < -v_max)
        v = -v_max;

    std::cout << "v: " << v << "   w: " << w << std::endl;
    std::cout << "(x,y,theta) = (" << x0 << "," << y0 << "," << theta << ")" << std::endl;
    std::cout << "------------------------------" << std::endl;

    // Publishする値の格納
    pub_msg.linear.x = v;
    pub_msg.linear.y = 0.0;
    pub_msg.linear.z = 0.0;
    pub_msg.angular.x = 0.0;
    pub_msg.angular.y = 0.0;
    pub_msg.angular.z = w; // 0?
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "karugamo");

    ros::NodeHandle nh;

    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_pub", 10);

    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odom_callback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    pos.pose.pose.position.x = 0.0;
    pos.pose.pose.position.y = 0.0;

    ros::Rate loop_rate(10);

    sleep(1);

    ros::spinOnce();

    std::cout << " robot will wait 5 seconds , stand front plz" << std::endl;
    sleep(5);

    while (ros::ok())
    {
        ros::spinOnce();

        if (min_leg_index == 1000)
        {
            std::cout << "syokai" << std::endl;
            setObject(default_rightindex, default_leftindex);
        }
        else
        {
            std::cout << "2 kai me ikou" << std::endl;
            setObject((min_leg_index - diff_index), (max_leg_index + diff_index));
        }

        // setObject(); // syokai ninsiki

        // if (isRecognized == 1)
        // {

        std::cout << "min_leg : " << min_leg_index << std::endl;
        std::cout << "max_leg : " << max_leg_index << std::endl;

        std::cout << "center 1 , x :" << pole1_cecnter[0];
        std::cout << " y : " << pole1_cecnter[1] << std::endl;

        std::cout << "center 2 , x :" << pole2_cecnter[0];
        std::cout << " y : " << pole2_cecnter[1] << std::endl;

        center_of_poles[0] = (pole1_cecnter[0] + pole2_cecnter[0]) / 2;
        center_of_poles[1] = (pole1_cecnter[1] + pole2_cecnter[1]) / 2;

        std::cout << "pole_center : x : " << center_of_poles[0] << ", y : " << center_of_poles[1] << std::endl;

        // isRecognized = 2;
        // }

        double distanceFromCenter = returnDistance(center_of_poles[0], center_of_poles[1]);

        // std::cout << "distanceFromCenter : " << distanceFromCenter << std::endl;

        if (distanceFromCenter > 0.5)
        {
            std::cout << "go forward" << std::endl;
            // moveFormard(pub, 0.3);
            // moveCtrl(pub, 0.3, 0);
            go_pos_GL(center_of_poles[0], center_of_poles[1]);
            pub.publish(pub_msg);
        }
        else if (0.4 < distanceFromCenter && distanceFromCenter < 0.5)
        {
            std::cout << "stop!" << std::endl;
            // moveStop(pub);
            moveCtrl(pub, 0, 0);
        }
        else
        {
            std::cout << "go BACK" << std::endl;
            // moveFormard(pub, -0.3);
            moveCtrl(pub, -0.3, 0);
        }

        // // rvizへとscanの値をそのままPublish
        scan_pub.publish(scan);

        isRecognized = 0;

        loop_rate.sleep();
    }
}