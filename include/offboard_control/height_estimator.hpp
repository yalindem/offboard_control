#ifndef CPP_HEIGHT_ESTIMATOR
#define CPP_HEIGHT_ESTIMATOR
#include "iostream"

namespace px4_offboard
{
    class HeightEstimator
    {
        public:
            HeightEstimator();
            void update_state(float baro_height, float imu_height, float imu_velo_z);
        
        private:
            
            float fused_height_{0.0f};
            const float KF_GAIN = 0.02f;
    };
}

#endif