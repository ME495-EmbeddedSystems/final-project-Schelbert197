/*********************************************************************
 Interactive Robotic Systems Lab
 Korea Advanced Institute of Science and Technology
 Prof. JeeHwan Ryu
 *********************************************************************
 \remarks		...
 \file		MovingAverageFilter.h
 \author	Joong-Ku Lee
 \date		MAR 03, 2022
 *********************************************************************/
#ifndef MOVINGAVERAGEFILTER_H
#define MOVINGAVERAGEFILTER_H

#include <Eigen/Dense>
#include <array>

using namespace Eigen;

class MovingAverageFilter
{
private:
    VectorXd input_, input_filtered_;
    double alpha_{0.0};

public:
    MovingAverageFilter(const int &numval, const double alpha)
    {
        alpha_ = alpha;
        VectorXd input(numval), input_filtered(numval);
        input.setZero();
        input_filtered.setZero();
        input_ = input;
        input_filtered_ = input_filtered;
    }

    void push(const VectorXd &newinput)
    {
        input_filtered_ = alpha_ * input_filtered_ + (1-alpha_) * newinput;
    }
    
    inline VectorXd getresult() const noexcept
    {
        return input_filtered_;
    }
};


#endif