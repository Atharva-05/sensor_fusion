/**
 * @file linear_kalman.cpp
 * @author Atharva Ghotavadekar (atharvagh1@gmail.com)
 * @brief Implementation of Kalman class
 * @version 0.1
 * 
 */

#include "linear_kalman.hpp"

Kalman::Kalman(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& P,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R)
    : A(A), P(P), Q(Q), R(R)
{
    // Initialize zero state vector (default behaviour)
    this->X = Eigen::MatrixXd::Zero(A.rows(), 1);

}

Kalman::Kalman(){}

void Kalman::initialize_filter(const Eigen::VectorXd& initial_state){
    // Initialize filter with non-zero initial state
    this->X = initial_state;

}

bool Kalman::update_state(const Eigen::MatrixXd A, const Eigen::VectorXd observation){

    // Predicting belief based on previous estimate
    this->X = this->A * this->X;
    this->P = ((this->A * this->P) * this->A.transpose()) + this->Q;

    // Computing Kalman Gain
    this->K_g = this->P * this->H.transpose() * ((this->H * this->P * this->H.transpose() + this->R).inverse());

    // Estimated state
    this->X = this->X + this->K_g * (observation - this->H * this->X);
    // Estimated state covariance
    this->P = (Eigen::MatrixXd::Identity(A.rows(), A.rows()) - this->K_g * this->H) * this->P;

    // Returns true on completion
    return true;

}