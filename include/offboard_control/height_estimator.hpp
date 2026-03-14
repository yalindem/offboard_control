#ifndef CPP_HEIGHT_ESTIMATOR
#define CPP_HEIGHT_ESTIMATOR
#include "iostream"
#include <memory>  
#include "kalman_filter.hpp"

namespace Drone::Estimator
{
    class HeightEstimator
    {

        public:
            HeightEstimator();

            void init(double initial_baro_height);
            void update_model(double u);
            void update_measurement(double z);
            double getHeight() const;
            double getVelo() const;
            double getBias() const;
        
        private:
            
            double fused_height_{0.0f};
            std::unique_ptr<Drone::Filter::KalmanFilter<3>> kf_;
    };
}

#endif