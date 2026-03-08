#ifndef HPP_KALMAN_FILTER
#define HPP_KALMAN_FILTER

#include <array>
#include <algorithm> // std::sort ve std::nth_element için gerekli
#include <Eigen/Dense>

namespace Drone::px4_offboard
{
    template<size_t R, size_t C>
    class KalmanFilter
    {       
        public:
            KalmanFilter(const Eigen::Matrix<double, R, C>& F, 
                         const Eigen::Matrix<double, R, 1>& B,
                         const Eigen::Matrix<double, R, R>& Q)
            {

            }

            template <typename... Args>
            void init(Args... init_values)
            {
                x_ << (init_values);
            }

            float update()
            {
                
            }

            void prediction()
            {

            }
        
        private:
            Eigen::Vector<R> x_;            // State vector
            Eigen::Matrix<double, R, C> F_; //State Transition Matrix
            Eigen::Matrix<double, R, 1> B_; //Control Input Matrix

            //w(k) Process Noise 
            //      IMU'nun gürültüsünü ve modelimizin mükemmel olmadığını temsil eder.
            //      Kovaryansı: Q matrix
            Eigen::Matrix<double, R, R> Q_; // Process Noise Covariance, Örnek: 
                                            // Height estimation: IMU'ma ve fizik modelime ne kadar güveniyorum
            Eigen::Matrix<double, R, 1> H_; // Observation Matrix
            Eigen::Matrix<double, 1, 1> R_; // Measurement Noise, Örnek: 
                                            // Height estimation: Barometreme ne kadar güveniyorum

            Eigen::Matrix<double, 1, 1> P_; // Error Covariance: Tahminimize ne kadar güvendiğimizin sayısal ifadesi.



    };
}

#endif