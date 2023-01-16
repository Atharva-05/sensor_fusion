/**
 * @file linear_kalman.hpp
 * @brief Header file for Linear Kalman Filter implementation
 * @version 0.1
 * 
 */

// Using Eigen Library for Matrix operations
#include <Eigen/Dense>

class Kalman{

    private:

        Eigen::MatrixXd X;      // State Vector
        Eigen::MatrixXd A;      // State transition
        Eigen::MatrixXd H;      // Measurement matrix
        Eigen::MatrixXd observation;

        Eigen::MatrixXd P;      // State Covariance Matrix
        Eigen::MatrixXd Q;      // Process noise covariance
        Eigen::MatrixXd R;      // Measurement noise covariance

        Eigen::MatrixXd K_g;    // Kalman Gain

        bool initialized;

    public:
        Kalman(
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& P,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R
        );

        Kalman();

        void initialize_filter(const Eigen::VectorXd& initial_state);

        bool update_state(const Eigen::MatrixXd A, const Eigen::VectorXd observation);

};