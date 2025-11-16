#include "offboard_control/height_estimator.hpp"

namespace px4_offboard
{
    HeightEstimator::HeightEstimator()
    {
        std::cout << "Height Estimator\n";
    }

    void HeightEstimator::update_state(float baro_height, float imu_height, float imu_velo_z)
    {
 
    }
}
