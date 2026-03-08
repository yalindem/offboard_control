#ifndef HPP_KALMAN_FILTER
#define HPP_KALMAN_FILTER

#include <array>
#include <algorithm>
#include <Eigen/Dense>

namespace Drone::px4_offboard
{
    template<size_t R, size_t C>
    class KalmanFilter
    {       
        public:
            KalmanFilter(const Eigen::Matrix<double, R, C>& F, 
                         const Eigen::Matrix<double, R, 1>& B,
                         const Eigen::Matrix<double, R, R>& Q,
                         const Eigen::Matrix<double, R, 1>) : F_(F), B_(B), Q_(Q), H_(H)
            {

            }

            void init(Eigen::Vector<double, R> init_x, Eigen::Matrix<double, 1, 1> init_P_)
            {
                x_ = init_x;
                P_ = init_P_;
            }

            void prediction(double u)
            {
                x_ = F_ * x_ + B_ * u;
                P_ = F * P_ * F.transpose() + Q_;
            }
            
            float update(double z)
            {
                double y = z - H_ * x_;
                Eigen::Matrix<double, 1, 1> S = H_ * P_ * H_.transpose() + R_;
                K_ = P_ * H_.transpose() * S_.inverse();
                x_ = x_ + K_ * y;
                P_ = (Eigen::Matrix<double, R, R>::Identity() - K_ * H_) * P_;
            }

            
        
        private:
            Eigen::Vector<double, R> x_;            // State vector
            Eigen::Matrix<double, R, C> F_;         //State Transition Matrix
            Eigen::Matrix<double, R, 1> B_;         //Control Input Matrix

            //w(k) Process Noise 
            //      IMU'nun gürültüsünü ve modelimizin mükemmel olmadığını temsil eder.
            //      Kovaryansı: Q matrix
            Eigen::Matrix<double, R, R> Q_;         // Process Noise Covariance, Örnek: 
                                                    // Height estimation: IMU'ma ve fizik modelime ne kadar güveniyorum
            Eigen::Matrix<double, R, 1> H_;         // Observation Matrix
            Eigen::Matrix<double, 1, 1> R_;         // Measurement Noise, Örnek: 
                                                    // Height estimation: Barometreme ne kadar güveniyorum

            Eigen::Matrix<double, 1, 1> P_;         // Error Covariance: Tahminimize ne kadar güvendiğimizin sayısal ifadesi.
            Eigen::Matrix<double, R, 1> K_;         // Kalman Gain



    };
}

#endif