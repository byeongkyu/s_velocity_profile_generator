#include "ros/ros.h"
#include <cstdlib>
#include <cmath>

#define EPSILON 0.0001

class SVelocityProfileGenerator
{
    public:
        SVelocityProfileGenerator()
        {
            is_trajectory_generated_ = false;
        }

        ~SVelocityProfileGenerator() {}

    public:
        bool generate_trajectory(double q0, double q1, double v0, double v1, double v_max, double a_max, double j_max)
        {
            is_trajectory_generated_ = false;;

            ROS_INFO("Planning Trajectory with given parameters: \n \
                        \t\t\t %f %f %f %f %f %f %f", q0, q1, v0, v1, v_max, a_max, j_max);

            // Search the seven segment time
            std::array<double, 5> output_params;
            if(!plan_trajectory({q0, q1, v0, v1, v_max, a_max, j_max}, output_params))
            {
                ROS_ERROR("Can't planning with given parameters.");
                return false;
            }

            // Save the result to member variables

            q0_ = q0;
            q1_ = q1;
            v0_ = v0;
            v1_ = v1;
            v_max_ = v_max;
            a_max_ = a_max;
            j_max_ = j_max;

            T_ = output_params[1] + output_params[3] + output_params[4];
            Tj1_ = output_params[0];
            Ta_ = output_params[1];
            Tj2_ = output_params[2];
            Td_ = output_params[3];
            Tv_ = output_params[4];

            a_lim_a_ = j_max * Tj1_;
            a_lim_d_ = -j_max * Tj2_;
            v_lim_ = v0 + (Ta_ - Tj1_) * a_lim_a_;

            is_trajectory_generated_ = true;

            // std::array<double, 7> args;
            // sign_transform({q0, q1, v0, v1, v_max, a_max, j_max}, args);

            // ROS_INFO("Sign transform result: \n
            //             \t\t\t\t %f %f %f %f %f %f %f", args[0], args[1], args[2], args[3], args[4], args[5], args[6]);


            return true;
        }

        bool get_accel_at(double t, double &accel)
        {
            if(!is_trajectory_generated_)
            {
                ROS_ERROR("Trajectory is not generated. try generate first...");
                return false;
            }

            return get_accel_in_trajectory(t, accel);
        }

        bool get_velocity_at(double t, double &vel)
        {
            if(!is_trajectory_generated_)
            {
                ROS_ERROR("Trajectory is not generated. try generate first...");
                return false;
            }

            return get_vel_in_trajectory(t, vel);;
        }

        bool get_position_at(double t, double &pos)
        {
            if(!is_trajectory_generated_)
            {
                ROS_ERROR("Trajectory is not generated. try generate first...");
                return false;
            }

            return get_pos_in_trajectory(t, pos);
        }

    private:
        bool sign_transform(std::array<double, 7> in, std::array<double, 7>& out)
        {
            double q0 = in[0];
            double q1 = in[1];

            double v0 = in[2];
            double v1 = in[3];

            double v_max = in[4];
            double a_max = in[5];
            double j_max = in[6];

            double v_min = -v_max;
            double a_min = -a_max;
            double j_min = -j_max;

            double s = copysign(1.0, (q1 - q0));
            double vs1 = (s + 1.0) / 2.0;
            double vs2 = (s - 1.0) / 2.0;

            out[0] = s * q0;    // q0
            out[1] = s * q1;    // q1
            out[2] = s * v0;
            out[3] = s * v1;
            out[4] = (vs1 * v_max) + (vs2 * v_min);
            out[5] = (vs1 * a_max) + (vs2 * a_min);
            out[6] = (vs1 * j_max) + (vs2 * j_min);

            return true;
        }

        bool check_is_feasible(std::array<double, 7> args)
        {
            double q0 = args[0];
            double q1 = args[1];
            double v0 = args[2];
            double v1 = args[3];
            double v_max = args[4];
            double a_max = args[5];
            double j_max = args[6];

            double dv = abs(v0 - v1);
            double dq = abs(q1 - q0);

            double time_to_reach_max_a = a_max / j_max;
            double time_to_set_speeds = sqrt(dv / j_max);

            double Tjs = std::min(time_to_reach_max_a, time_to_set_speeds);

            if(Tjs == time_to_reach_max_a)
            {
                return (dq > (0.5 * (v0 + v1) * (Tjs + (dv / a_max))));
            }
            else if(Tjs < time_to_reach_max_a)
            {
                return (dq > (Tjs * (v0 + v1)));
            }
            else
            {
                throw std::runtime_error("planning error");
            }

            return false;
        }

        bool compute_maximum_speed_reached(std::array<double, 7> args, std::array<double, 5>& results)
        {
            double q0 = args[0];
            double q1 = args[1];
            double v0 = args[2];
            double v1 = args[3];
            double v_max = args[4];
            double a_max = args[5];
            double j_max = args[6];

            // acceleration period
            double Tj1 = 0.0;
            double Ta = 0.0;
            if(((v_max - v0) * j_max) < pow(a_max, 2))
            {
                Tj1 = sqrt((v_max - v0) / j_max);
                Ta = 2.0 * Tj1;
            }
            else
            {
                Tj1 = a_max / j_max;
                Ta = Tj1 + (v_max - v0) / a_max;
            }

            // deceleration period
            double Tj2 = 0.0;
            double Td = 0.0;
            if(((v_max - v1) * j_max) < pow(a_max, 2))
            {
                Tj2 = sqrt((v_max - v1) / j_max);
                Td = 2.0 * Tj2;
            }
            else
            {
                Tj2 = a_max / j_max;
                Td = Tj2 + (v_max - v1) / a_max;
            }

            double Tv = ((q1 - q0) / v_max)
                        - ((Ta / 2.0) * (1.0 + v0 / v_max))
                        - ((Td / 2.0) *  (1.0 + v1 / v_max));

            if(Tv < 0.0)
            {
                return false;
            }

            results[0] = Tj1;
            results[1] = Ta;
            results[2] = Tj2;
            results[3] = Td;
            results[4] = Tv;

            return true;
        }

        bool compute_maximum_speed_not_reached(std::array<double, 7> args, std::array<double, 5>& results)
        {
            double q0 = args[0];
            double q1 = args[1];
            double v0 = args[2];
            double v1 = args[3];
            double v_max = args[4];
            double a_max = args[5];
            double j_max = args[6];

            double Tj1 = a_max / j_max;
            double Tj2 = a_max / j_max;
            double Tj = a_max / j_max;
            double Tv = 0.0;

            double v = pow(a_max, 2) / j_max;
            double delta = (pow(a_max, 4) / pow(j_max, 2))
                        + 2.0 * (pow(v0, 2) + pow(v1, 2))
                        + a_max * (4.0 * (q1 - q0) - 2.0 * (a_max / j_max) * (v0 + v1));

            double Ta = (v - 2.0 * v0 + sqrt(delta)) / (2.0 * a_max);
            double Td = (v - 2.0 * v1 + sqrt(delta)) / (2.0 * a_max);

            if((Ta - 2.0 * Tj) < EPSILON || (Td - 2.0 * Tj) < EPSILON)
            {
                return false;
            }

            results[0] = Tj1;
            results[1] = Ta;
            results[2] = Tj2;
            results[3] = Td;
            results[4] = Tv;

            return true;
        }

        bool scurve_search_planning(std::array<double, 7> args, std::array<double, 5>& results)
        {
            double q0 = args[0];
            double q1 = args[1];
            double v0 = args[2];
            double v1 = args[3];
            double v_max = args[4];
            double a_max = args[5];
            double j_max = args[6];

            double l = 0.99;
            int max_iter = 2000;
            double dt_thresh = 0.01;

            double _a_max = a_max;
            int it = 0;

            while((it < max_iter) && (_a_max > EPSILON))
            {
                std::array<double, 5> iter_result;
                if(compute_maximum_speed_not_reached({q0, q1, v0, v1, v_max, _a_max, j_max}, iter_result))
                {
                    results[0] = iter_result[0];
                    results[1] = iter_result[1];
                    results[2] = iter_result[2];
                    results[3] = iter_result[3];
                    results[4] = iter_result[4];

                    return true;
                }
                else
                {
                    it++;
                    _a_max *= l;
                }
            }

            return false;
        }

        bool plan_trajectory(std::array<double, 7> in_params, std::array<double, 5> out_params)
        {
            double q0 = in_params[0];
            double q1 = in_params[1];
            double v0 = in_params[2];
            double v1 = in_params[3];
            double v_max = in_params[4];
            double a_max = in_params[5];
            double j_max = in_params[6];

            std::array<double, 5> plan_results;
            if(check_is_feasible({q0, q1, v0, v1, v_max, a_max, j_max}))
            {
                ROS_INFO("This trajectory is feasible...");
                if(compute_maximum_speed_reached({q0, q1, v0, v1, v_max, a_max, j_max}, plan_results))
                {
                    ROS_INFO("compute_maximum_speed_reached: \n \
                                \t\t Tj1: %f,  Ta: %f,  Tj2: %f,  Td: %f,  Tv: %f",
                                plan_results[0], plan_results[1], plan_results[2], plan_results[3], plan_results[4]);
                }
                else if(compute_maximum_speed_not_reached({q0, q1, v0, v1, v_max, a_max, j_max}, plan_results))
                {
                    ROS_INFO("compute_maximum_speed_not_reached: \n \
                                \t\t Tj1: %f,  Ta: %f,  Tj2: %f,  Td: %f,  Tv: %f",
                                plan_results[0], plan_results[1], plan_results[2], plan_results[3], plan_results[4]);
                }
                else if(scurve_search_planning({q0, q1, v0, v1, v_max, a_max, j_max}, plan_results))
                {
                    ROS_INFO("scurve_search_planning: \n \
                                \t\t Tj1: %f,  Ta: %f,  Tj2: %f,  Td: %f,  Tv: %f",
                                plan_results[0], plan_results[1], plan_results[2], plan_results[3], plan_results[4]);
                }
                else
                {
                    ROS_INFO("This trajectory seems infeasible...");
                    return false;
                }
            }
            else
            {
                ROS_INFO("This trajectory is not feasible...");
                return false;
            }

            ROS_INFO("Planning results: T: %f, Tj1: %f, Ta: %f, Tj2: %f, Td: %f, Tv: %f",
                (plan_results[1] + plan_results[3] + plan_results[4]),
                plan_results[0],
                plan_results[1],
                plan_results[2],
                plan_results[3],
                plan_results[4]
            );

            out_params[0] = plan_results[0];
            out_params[1] = plan_results[1];
            out_params[2] = plan_results[2];
            out_params[3] = plan_results[3];
            out_params[4] = plan_results[4];

            return true;
        }

        bool get_accel_in_trajectory(double t, double &accel)
        {
            // Phase 0
            if(t >= 0.0 && t < Tj1_)
            {
                accel = j_max_ * t;
            }
            // Phase 1
            else if(t >= Tj1_ && t < (Ta_ - Tj1_))
            {
                accel = a_lim_a_;
            }
            // Phase 2
            else if(t >= (Ta_ - Tj1_) && t < Ta_)
            {
                double tt = Ta_ - t;
                accel = j_max_ * tt;
            }
            // Phase 3
            else if(t >= Ta_ && t < (Ta_ + Tv_))
            {
                accel = 0.0;
            }
            // Phase 4
            else if(t >= (T_ - Td_) && t < (T_ - Td_ + Tj2_))
            {
                double tt = t - T_ + Td_;
                accel = -j_max_ * tt;
            }
            // Phase 5
            else if(t >= (T_ - Td_ + Tj2_) && t < (T_ - Tj2_))
            {
                double tt = t - T_ + Td_;
                accel = a_lim_d_;
            }
            // Phase 6
            else if(t >= (T_ - Tj2_) && t < T_)
            {
                double tt = T_ - t;
                accel = -j_max_ * tt;
            }
            // Other case
            else
            {
                accel = 0.0;
            }
            return true;
        }

        bool get_vel_in_trajectory(double t, double &vel)
        {
            // Phase 0
            if(t >= 0.0 && t < Tj1_)
            {
                vel = v0_ + j_max_ * pow(t, 2) / 2.0;
            }
            // Phase 1
            else if(t >= Tj1_ && t < (Ta_ - Tj1_))
            {
                vel = v0_ + a_lim_a_ * (t - Tj1_ / 2.0);
            }
            // Phase 2
            else if(t >= (Ta_ - Tj1_) && t < Ta_)
            {
                double tt = Ta_ - t;
                vel = v_lim_ - j_max_ * pow(tt, 2) / 2.0;
            }
            // Phase 3
            else if(t >= Ta_ && t < (Ta_ + Tv_))
            {
                vel = v_lim_;
            }
            // Phase 4
            else if(t >= (T_ - Td_) && t < (T_ - Td_ + Tj2_))
            {
                double tt = t - T_ + Td_;
                vel = v_lim_ - j_max_ * pow(tt, 2) / 2.0;
            }
            // Phase 5
            else if(t >= (T_ - Td_ + Tj2_) && t < (T_ - Tj2_))
            {
                double tt = t - T_ + Td_;
                vel = v_lim_ + a_lim_d_ * (tt - Tj2_ / 2.0);
            }
            // Phase 6
            else if(t >= (T_ - Tj2_) && t < T_)
            {
                double tt = T_ - t;
                vel = v1_ + j_max_ * pow(tt, 2) / 2.0;
            }
            // Other case
            else
            {
                vel = v1_;
            }
            return true;
        }

        bool get_pos_in_trajectory(double t, double &pos)
        {
            // Phase 0
            if(t >= 0.0 && t < Tj1_)
            {
                pos = q0_ + v0_ * t + j_max_ * pow(t, 3) / 6.0;
            }
            // Phase 1
            else if(t >= Tj1_ && t < (Ta_ - Tj1_))
            {
                pos = q0_ + v0_ * t + a_lim_a_ * (3.0 * pow(t, 2) - 3.0 * Tj1_ * t + pow(Tj1_, 2)) / 6.0;
            }
            // Phase 2
            else if(t >= (Ta_ - Tj1_) && t < Ta_)
            {
                double tt = Ta_ - t;
                pos = q0_ + (v_lim_ + v0_) * Ta_ / 2.0 - v_lim_ * tt + j_max_ * pow(tt, 3) / 6.0;
            }
            // Phase 3
            else if(t >= Ta_ && t < (Ta_ + Tv_))
            {
                pos = q0_ + (v_lim_ + v0_) * Ta_ / 2.0 + v_lim_ * (t - Ta_);
            }
            // Phase 4
            else if(t >= (T_ - Td_) && t < (T_ - Td_ + Tj2_))
            {
                double tt = t - T_ + Td_;
                pos = q1_ - (v_lim_ + v1_) * Td_ / 2.0 + v_lim_ * tt - j_max_ * pow(tt, 3) / 6.0;
            }
            // Phase 5
            else if(t >= (T_ - Td_ + Tj2_) && t < (T_ - Tj2_))
            {
                double tt = t - T_ + Td_;
                pos = q1_ - (v_lim_ + v1_) * Td_ / 2.0 + v_lim_ * tt
                        + a_lim_d_ * (3.0 * pow(tt, 2) - 3.0 * Tj2_ * tt + pow(Tj2_, 2))  / 6.0;
            }
            // Phase 6
            else if(t >= (T_ - Tj2_) && t < T_)
            {
                double tt = T_ - t;
                pos = q1_ - v1_ * tt - j_max_ * pow(tt, 3) / 6.0;
            }
            // Other case
            else
            {
                pos = q1_;
            }
            return true;
        }

    private:
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
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_profile_node");

    if(argc != 8)
    {
        std::cerr << "Usage: " << argv[0] << " <q0> <q1> <v0> <v1> <v_max> <a_max> <j_max>" << std::endl;
        return -1;
    }

    double q0 = std::atof(argv[1]);
    double q1 = std::atof(argv[2]);
    double v0 = std::atof(argv[3]);
    double v1 = std::atof(argv[4]);
    double v_max = std::atof(argv[5]);
    double a_max = std::atof(argv[6]);
    double j_max = std::atof(argv[7]);


    SVelocityProfileGenerator m;
    m.generate_trajectory(q0, q1, v0, v1, v_max, a_max, j_max);   // q0, q1, v0, v1, v_max, a_max, j_max

    for(double t = 0.0; t < 3.2; t += 0.02)
    {
        double a = 0;
        double v = 0;
        double p = 0;

        m.get_accel_at(t, a);
        m.get_velocity_at(t, v);
        m.get_position_at(t, p);

        ROS_INFO("%f %f %f %f", t, a, v, p);
    }

    ros::spin();
    return 0;
}