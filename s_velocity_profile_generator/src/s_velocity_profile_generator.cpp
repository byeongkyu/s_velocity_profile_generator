#include "s_velocity_profile_generator/s_velocity_profile_generator.h"

SVelocityProfileGenerator::SVelocityProfileGenerator()
:  EPSILON(0.0001)
{
    reset();
}

SVelocityProfileGenerator::~SVelocityProfileGenerator()
{
}

void SVelocityProfileGenerator::reset()
{
    is_trajectory_generated_ = false;
    q0_ = 0.0;
    q1_ = 0.0;
    v0_ = 0.0;
    v1_ = 0.0;
    v_max_ = 0.0;
    a_max_ = 0.0;
    j_max_ = 0.0;

    T_ = 0.0;
    Tj1_ = 0.0;
    Ta_ = 0.0;
    Tj2_ = 0.0;
    Td_ = 0.0;
    Tv_ = 0.0;
    a_lim_a_ = 0.0;
    a_lim_d_ = 0.0;
    v_lim_ = 0.0;

    sign_ = 0.0;
    phase_accel_sign_ = 0.0;
}

bool SVelocityProfileGenerator::generate_trajectory(double q0, double q1, double v0, double v1, double v_max, double a_max, double j_max)
{
    is_trajectory_generated_ = false;

    std::array<double, 7> out_sign;
    sign_transform({q0, q1, v0, v1, v_max, a_max, j_max}, out_sign);

    ROS_DEBUG("Planning Trajectory with given parameters: \n \
                \t\t\t %f %f %f %f %f %f %f", out_sign[0], out_sign[1], out_sign[2], out_sign[3], out_sign[4], out_sign[5], out_sign[6]);

    // Search the seven segment time
    std::array<double, 5> output_params;
    if(!plan_trajectory({out_sign[0], out_sign[1], out_sign[2], out_sign[3], out_sign[4], out_sign[5], out_sign[6]}, output_params))
    {
        ROS_ERROR("Can't planning with given parameters.");
        return false;
    }

    // Save the result to member variables
    ROS_DEBUG("Save the params: %f %f %f %f %f", output_params[0], output_params[1], output_params[2], output_params[3], output_params[4]);

    q0_ = out_sign[0];
    q1_ = out_sign[1];
    v0_ = out_sign[2];
    v1_ = out_sign[3];
    v_max_ = out_sign[4];
    a_max_ = out_sign[5];
    j_max_ = out_sign[6];

    T_ = output_params[1] + output_params[3] + output_params[4];
    Tj1_ = output_params[0];
    Ta_ = output_params[1];
    Tj2_ = output_params[2];
    Td_ = output_params[3];
    Tv_ = output_params[4];

    a_lim_a_ = j_max * Tj1_;
    a_lim_d_ = -j_max * Tj2_;
    v_lim_ = abs(v0) + phase_accel_sign_ * (Ta_ - Tj1_) * a_lim_a_;

    is_trajectory_generated_ = true;
    return true;
}

bool SVelocityProfileGenerator::get_accel_at(double t, double &accel)
{
    if(!is_trajectory_generated_)
    {
        ROS_ERROR("Trajectory is not generated. try generate first...");
        return false;
    }

    return get_accel_in_trajectory(t, accel);
}

bool SVelocityProfileGenerator::get_velocity_at(double t, double &vel)
{
    if(!is_trajectory_generated_)
    {
        ROS_ERROR("Trajectory is not generated. try generate first...");
        return false;
    }

    return get_vel_in_trajectory(t, vel);;
}

bool SVelocityProfileGenerator::get_position_at(double t, double &pos)
{
    if(!is_trajectory_generated_)
    {
        ROS_ERROR("Trajectory is not generated. try generate first...");
        return false;
    }

    return get_pos_in_trajectory(t, pos);
}

void SVelocityProfileGenerator::sign_transform(std::array<double, 7> in, std::array<double, 7>& out)
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

    sign_ = copysign(1.0, (q1 - q0));
    double vs1 = (sign_ + 1.0) / 2.0;
    double vs2 = (sign_ - 1.0) / 2.0;

    out[0] = sign_ * q0;    // q0
    out[1] = sign_ * q1;    // q1
    out[2] = sign_ * v0;
    out[3] = sign_ * v1;
    out[4] = (vs1 * v_max) + (vs2 * v_min);
    out[5] = (vs1 * a_max) + (vs2 * a_min);
    out[6] = (vs1 * j_max) + (vs2 * j_min);
}

bool SVelocityProfileGenerator::check_is_feasible(std::array<double, 7> args)
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

bool SVelocityProfileGenerator::compute_maximum_speed_reached(std::array<double, 7> args, std::array<double, 5>& results)
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

    phase_accel_sign_ = copysign(1.0, (v_max - v0));
    if((abs(v_max - v0) * j_max) < pow(a_max, 2))
    {
        Tj1 = sqrt(abs(v_max - v0) / j_max);
        Ta = 2.0 * Tj1;
    }
    else
    {
        Tj1 = a_max / j_max;
        Ta = Tj1 + abs(v_max - v0) / a_max;
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

bool SVelocityProfileGenerator::compute_maximum_speed_not_reached(std::array<double, 7> args, std::array<double, 5>& results)
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

bool SVelocityProfileGenerator::scurve_search_planning(std::array<double, 7> args, std::array<double, 5>& results)
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

bool SVelocityProfileGenerator::plan_trajectory(std::array<double, 7> in_params, std::array<double, 5>& out_params)
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
        ROS_DEBUG("This trajectory is feasible...");
        if(compute_maximum_speed_reached({q0, q1, v0, v1, v_max, a_max, j_max}, plan_results))
        {
            ROS_DEBUG("compute_maximum_speed_reached: \n \
                        \t\t Tj1: %f,  Ta: %f,  Tj2: %f,  Td: %f,  Tv: %f",
                        plan_results[0], plan_results[1], plan_results[2], plan_results[3], plan_results[4]);
        }
        else if(compute_maximum_speed_not_reached({q0, q1, v0, v1, v_max, a_max, j_max}, plan_results))
        {
            ROS_DEBUG("compute_maximum_speed_not_reached: \n \
                        \t\t Tj1: %f,  Ta: %f,  Tj2: %f,  Td: %f,  Tv: %f",
                        plan_results[0], plan_results[1], plan_results[2], plan_results[3], plan_results[4]);
        }
        else if(scurve_search_planning({q0, q1, v0, v1, v_max, a_max, j_max}, plan_results))
        {
            ROS_DEBUG("scurve_search_planning: \n \
                        \t\t Tj1: %f,  Ta: %f,  Tj2: %f,  Td: %f,  Tv: %f",
                        plan_results[0], plan_results[1], plan_results[2], plan_results[3], plan_results[4]);
        }
        else
        {
            ROS_ERROR("This trajectory seems infeasible...");
            return false;
        }
    }
    else
    {
        ROS_ERROR("This trajectory is not feasible...");
        return false;
    }

    ROS_DEBUG("Planning results: T: %f, Tj1: %f, Ta: %f, Tj2: %f, Td: %f, Tv: %f",
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

bool SVelocityProfileGenerator::get_accel_in_trajectory(double t, double &accel)
{
    double ret_val = 0.0;

    // Phase 0
    if(t >= 0.0 && t < Tj1_)
    {
        ret_val = (j_max_ * phase_accel_sign_) * t;
    }
    // Phase 1
    else if(t >= Tj1_ && t < (Ta_ - Tj1_))
    {
        ret_val = phase_accel_sign_ * a_lim_a_;
    }
    // Phase 2
    else if(t >= (Ta_ - Tj1_) && t < Ta_)
    {
        double tt = Ta_ - t;
        ret_val = (j_max_ * phase_accel_sign_) * tt;
    }
    // Phase 3
    else if(t >= Ta_ && t < (Ta_ + Tv_))
    {
        ret_val = 0.0;
    }
    // Phase 4
    else if(t >= (T_ - Td_) && t < (T_ - Td_ + Tj2_))
    {
        double tt = t - T_ + Td_;
        ret_val = -j_max_ * tt;
    }
    // Phase 5
    else if(t >= (T_ - Td_ + Tj2_) && t < (T_ - Tj2_))
    {
        double tt = t - T_ + Td_;
        ret_val = a_lim_d_;
    }
    // Phase 6
    else if(t >= (T_ - Tj2_) && t < T_)
    {
        double tt = T_ - t;
        ret_val = -j_max_ * tt;
    }
    // Other case
    else
    {
        ret_val = 0.0;
    }

    accel = ret_val * sign_;
    return true;
}

bool SVelocityProfileGenerator::get_vel_in_trajectory(double t, double &vel)
{
    double ret_val = 0.0;

    // Phase 0
    if(t >= 0.0 && t < Tj1_)
    {
        ret_val = v0_ + (j_max_ * phase_accel_sign_) * pow(t, 2) / 2.0;
    }
    // Phase 1
    else if(t >= Tj1_ && t < (Ta_ - Tj1_))
    {
        ret_val = v0_ + (phase_accel_sign_ * a_lim_a_) * (t - Tj1_ / 2.0);
    }
    // Phase 2
    else if(t >= (Ta_ - Tj1_) && t < Ta_)
    {
        double tt = Ta_ - t;
        ret_val = v_lim_ - (j_max_ * phase_accel_sign_) * pow(tt, 2) / 2.0;
    }
    // Phase 3
    else if(t >= Ta_ && t < (Ta_ + Tv_))
    {
        ret_val = v_lim_;
    }
    // Phase 4
    else if(t >= (T_ - Td_) && t < (T_ - Td_ + Tj2_))
    {
        double tt = t - T_ + Td_;
        ret_val = v_lim_ - j_max_ * pow(tt, 2) / 2.0;
    }
    // Phase 5
    else if(t >= (T_ - Td_ + Tj2_) && t < (T_ - Tj2_))
    {
        double tt = t - T_ + Td_;
        ret_val = v_lim_ + a_lim_d_ * (tt - Tj2_ / 2.0);
    }
    // Phase 6
    else if(t >= (T_ - Tj2_) && t < T_)
    {
        double tt = T_ - t;
        ret_val = v1_ + j_max_ * pow(tt, 2) / 2.0;
    }
    // Other case
    else
    {
        ret_val = v1_;
    }

    vel = ret_val * sign_;
    return true;
}

bool SVelocityProfileGenerator::get_pos_in_trajectory(double t, double &pos)
{
    double ret_val = 0.0;

    // Phase 0
    if(t >= 0.0 && t < Tj1_)
    {
        ret_val = q0_ + v0_ * t + j_max_ * pow(t, 3) / 6.0;
    }
    // Phase 1
    else if(t >= Tj1_ && t < (Ta_ - Tj1_))
    {
        ret_val = q0_ + v0_ * t + (phase_accel_sign_ * a_lim_a_) * (3.0 * pow(t, 2) - 3.0 * Tj1_ * t + pow(Tj1_, 2)) / 6.0;
    }
    // Phase 2
    else if(t >= (Ta_ - Tj1_) && t < Ta_)
    {
        double tt = Ta_ - t;
        ret_val = q0_ + (v_lim_ + v0_) * Ta_ / 2.0 - v_lim_ * tt + j_max_ * pow(tt, 3) / 6.0;
    }
    // Phase 3
    else if(t >= Ta_ && t < (Ta_ + Tv_))
    {
        ret_val = q0_ + (v_lim_ + v0_) * Ta_ / 2.0 + v_lim_ * (t - Ta_);
    }
    // Phase 4
    else if(t >= (T_ - Td_) && t < (T_ - Td_ + Tj2_))
    {
        double tt = t - T_ + Td_;
        ret_val = q1_ - (v_lim_ + v1_) * Td_ / 2.0 + v_lim_ * tt - j_max_ * pow(tt, 3) / 6.0;
    }
    // Phase 5
    else if(t >= (T_ - Td_ + Tj2_) && t < (T_ - Tj2_))
    {
        double tt = t - T_ + Td_;
        ret_val = q1_ - (v_lim_ + v1_) * Td_ / 2.0 + v_lim_ * tt
                + a_lim_d_ * (3.0 * pow(tt, 2) - 3.0 * Tj2_ * tt + pow(Tj2_, 2))  / 6.0;
    }
    // Phase 6
    else if(t >= (T_ - Tj2_) && t < T_)
    {
        double tt = T_ - t;
        ret_val = q1_ - v1_ * tt - j_max_ * pow(tt, 3) / 6.0;
    }
    // Other case
    else
    {
        ret_val = q1_;
    }

    pos = ret_val * sign_;
    return true;
}