#include "LpfThreshold.h"

LpfThreshold::LpfThreshold(void) {}

void LpfThreshold::setValues(double offset, double filtering, int jointNumber) {
    offset_ = offset;
    filtering_ = filtering;
    jointNumber_ = jointNumber;
    filtered_signal_b_ = 0.0;
    filtered_signal_ = Eigen::VectorXd::Zero(jointNumber_);
}

double LpfThreshold::adaptiveThreshold(double newSignal, bool high) {
    filtered_signal_b_ = filtering_ * newSignal + (1 - filtering_) * filtered_signal_b_;
    return high ? filtered_signal_b_ + offset_: filtered_signal_b_ - offset_;
}

Eigen::VectorXd LpfThreshold::adaptiveThreshold(const Eigen::VectorXd& newSignal, bool high) {
    filtered_signal_ = filtering_ * newSignal + (1 - filtering_) * filtered_signal_;
    if(high)
    {
        return filtered_signal_ + Eigen::VectorXd::Constant(jointNumber_, offset_);
    }
    else
    {
        return filtered_signal_ - Eigen::VectorXd::Constant(jointNumber_, offset_);
    }
}

void LpfThreshold::setOffset(double offset) {
    offset_ = offset;
}

void LpfThreshold::setFiltering(double filtering) {
    filtering_ = filtering;
}