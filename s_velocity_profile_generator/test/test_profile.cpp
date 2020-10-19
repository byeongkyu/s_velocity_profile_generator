#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "s_velocity_profile_generator/s_velocity_profile_generator.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_profile_node");
    ros::NodeHandle nh;

    if(argc != 8)
    {
        std::cerr << "Usage: " << argv[0] << " <q0> <q1> <v0> <v1> <v_max> <a_max> <j_max>" << std::endl;
        return -1;
    }

    ros::Publisher pub_pos = nh.advertise<std_msgs::Float64>("result_pos", 1000);
    ros::Publisher pub_vel = nh.advertise<std_msgs::Float64>("result_vel", 1000);
    ros::Publisher pub_acc = nh.advertise<std_msgs::Float64>("result_acc", 1000);

    ros::Duration(0.5).sleep();

    double q0 = std::atof(argv[1]);
    double q1 = std::atof(argv[2]);
    double v0 = std::atof(argv[3]);
    double v1 = std::atof(argv[4]);
    double v_max = std::atof(argv[5]);
    double a_max = std::atof(argv[6]);
    double j_max = std::atof(argv[7]);


    SVelocityProfileGenerator m;
    m.generate_trajectory(q0, q1, v0, v1, v_max, a_max, j_max);   // q0, q1, v0, v1, v_max, a_max, j_max

    double t = 0.0;
    while(ros::ok())
    {
        std_msgs::Float64 pos_data;
        std_msgs::Float64 vel_data;
        std_msgs::Float64 acc_data;

        double a = 0;
        double v = 0;
        double p = 0;

        m.get_accel_at(t, a);
        m.get_velocity_at(t, v);
        m.get_position_at(t, p);

        // ROS_INFO("%f %f %f %f", t, a, v, p);
        pos_data.data = p;
        vel_data.data = v;
        acc_data.data = a;

        pub_pos.publish(pos_data);
        pub_vel.publish(vel_data);
        pub_acc.publish(acc_data);

        ros::Duration(0.02).sleep();
        t = t + 0.02;
        if(p == q1)
            break;
    }

    ros::spin();
    return 0;
}