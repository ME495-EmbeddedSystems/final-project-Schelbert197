/*********************************************************************
 Interactive Robotic Systems Lab
 Korea Advanced Institute of Science and Technology
 Prof. JeeHwan Ryu
 *********************************************************************
 \remarks		...
 \file		JointLimitAvoidance.h
 \author	Joong-Ku Lee
 \date		FEB 24, 2022
 *********************************************************************/

#ifndef PANDA_TELEOPERATION_DRIVER_SRC_JOINTLIMITAVOIDANCE_H
#define PANDA_TELEOPERATION_DRIVER_SRC_JOINTLIMITAVOIDANCE_H

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;


/// @brief Using the joint limits for the FRANKA EMIKA RESEARCH 3
/// https://frankaemika.github.io/docs/control_parameters.html#limits-for-franka-research-3
///
class JointLimitAvoidance
{
    VectorXd q_max_, q_min_, dq_max_;

public:
    JointLimitAvoidance(const string &str)
    {
        VectorXd q_max(7), q_min(7), dq_max(7);
        if (str == "right"){
            q_max << 2.7437, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
            q_min << -2.0, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        } else if (str == "left"){
            q_max << M_PI/2, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
            q_min << 0, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        } else {
            q_max << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
            q_min << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        }
        
        dq_max << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;

        q_max_ = q_max;
        q_min_ = q_min;
        dq_max_ = dq_max;
    }

    VectorXd JointPositionLimitAvoidance(const VectorXd &q)
    {
        VectorXd pos_torque(7);

        const double dist = 0.15;
        const double upper_threshold = 10.0;
        const double exp_coef = 30;

        for (int i = 0; i < q.size(); ++i)
        {
            pos_torque(i) = upper_threshold * (1/(1+exp(exp_coef * (q(i) - (q_min_(i) + dist)))) + 1/(1+exp(exp_coef * (q(i) - (q_max_(i) - dist))))) - upper_threshold;
        }

        return pos_torque;
    }

    VectorXd JointVelocityLimitAvoidance(const VectorXd &dq)
    {
        VectorXd vel_torque(7);

        const double dist = 0.2;
        const double upper_threshold = 5.0;
        const double exp_coef = 30;

        for (int i = 0; i < dq.size(); ++i)
        {
            vel_torque(i) = upper_threshold * (1/(1+exp(exp_coef * (dq(i) - (-dq_max_(i) + dist)))) + 1/(1+exp(exp_coef * (dq(i) - (dq_max_(i) - dist))))) - upper_threshold;
        }

        return vel_torque;
    }

    VectorXd CalculateTorque(const VectorXd &q, const VectorXd &dq)
    {
        VectorXd torque(7), pos_torque(7), vel_torque(7);

        pos_torque = JointPositionLimitAvoidance(q);
        vel_torque = JointVelocityLimitAvoidance(dq);

        torque = pos_torque + vel_torque;

        return torque;
    }

};

#endif