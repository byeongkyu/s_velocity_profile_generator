#ifndef S_VELOCITY_PROFILE_GENERATOR_H_
#define S_VELOCITY_PROFILE_GENERATOR_H_

#include "ros/ros.h"
#include <cstdlib>
#include <cmath>

class SVelocityProfileGenerator
{
    public:
        SVelocityProfileGenerator();
        virtual ~SVelocityProfileGenerator();

    public:
        bool generate_trajectory(double q0, double q1, double v0, double v1, double v_max, double a_max, double j_max);
        bool get_accel_at(double t, double &accel);
        bool get_velocity_at(double t, double &vel);
        bool get_position_at(double t, double &pos);
        void reset();

    private:
        void sign_transform(std::array<double, 7> in, std::array<double, 7>& out);
        bool check_is_feasible(std::array<double, 7> args);
        bool compute_maximum_speed_reached(std::array<double, 7> args, std::array<double, 5>& results);
        bool compute_maximum_speed_not_reached(std::array<double, 7> args, std::array<double, 5>& results);
        bool scurve_search_planning(std::array<double, 7> args, std::array<double, 5>& results);
        bool plan_trajectory(std::array<double, 7> in_params, std::array<double, 5>& out_params);
        bool get_accel_in_trajectory(double t, double &accel);
        bool get_vel_in_trajectory(double t, double &vel);
        bool get_pos_in_trajectory(double t, double &pos);

    private:
        const double EPSILON;
        bool is_trajectory_generated_;
        double q0_;
        double q1_;
        double v0_;
        double v1_;
        double v_max_;
        double a_max_;
        double j_max_;

        double T_;
        double Tj1_;
        double Ta_;
        double Tj2_;
        double Td_;
        double Tv_;
        double a_lim_a_;
        double a_lim_d_;
        double v_lim_;

        double sign_;
        double phase_accel_sign_;
};

#endif //S_VELOCITY_PROFILE_GENERATOR_H_