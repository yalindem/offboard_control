#include "offboard_control/height_estimator.hpp"

namespace Drone::Estimator
{
    HeightEstimator::HeightEstimator()
    {

        std::cout << "Height Estimator\n";
        
    }


    void HeightEstimator::init(double initial_baro_height)
    {   
        constexpr double dt = 1.0 / 200.0;  // 0.005 s, IMU

        // ─── F — State Transition Matrix (3×3) ───────────────────────────────────────
        // State: x = [z, z_dot, bias]^T
        // Kinematik ilişki:
        //   z(k)     = z(k-1)     + dt·ż(k-1) - (dt²/2)·b(k-1)
        //   ż(k)     = ż(k-1)                 - dt·b(k-1)
        //   bias(k)  = bias(k-1)                               ← random walk
        Eigen::Matrix3d F;

        F << 1.0,  dt,  -0.5 * dt * dt,
            0.0,  1.0,       -dt,
            0.0,  0.0,        1.0;
        

        // ─── B — Control Input Matrix (3×1) ──────────────────────────────────────────
        // u = a_measured (IMU'dan gelen dikey ivme, gravity çıkarılmış)
        // IMU ivmesi state'leri şöyle etkiler:
        //   z     += (dt²/2) · u
        //   ż     +=  dt     · u
        //   bias  +=  0      (ivme bias'ı doğrudan etkilemez)
        Eigen::Vector3d B;
        B << 0.5 * dt * dt,
            dt,
            0.0;
        
        // ─── Q — Process Noise Covariance (3×3) ──────────────────────────────────────
        // IMU noise density: tipik MEMS için ~150 μg/√Hz (varsayılan)
        // sigma_a = noise_density × sqrt(IMU_freq)
        //         = 150e-6 × 9.81 × sqrt(200) ≈ 0.00208 m/s²
        // Q = sigma_a² × G×Gᵀ  +  bias random walk terimi
        //
        // G: ivmenin state'lere coupling vektörü (B ile aynı)
        constexpr double noise_density = 150e-6 * 9.81;          // m/s²/√Hz
        constexpr double sigma_a       = noise_density * std::sqrt(200.0);
        constexpr double sigma_a2      = sigma_a * sigma_a;

        constexpr double bias_rw       = 1e-4;   // bias random walk: m/s² per √s
                                                // uçuşta ısınma vs. için küçük tutulur
        Eigen::Vector3d G;
        G << 0.5 * dt * dt,
                dt,
                0.0;

        Eigen::Matrix3d Q = sigma_a2 * G * G.transpose();
        Q(2, 2) += bias_rw * bias_rw * dt;      // bias random walk diagonal'a eklenir
        
        // ─── H — Observation Matrix (1×3) ────────────────────────────────────────────
        // Barometer sadece yüksekliği ölçer → [1, 0, 0]
        Eigen::Matrix<double, 1, 3> H;
        H << 1.0, 0.0, 0.0;


        // ─── R — Measurement Noise (1×1) ─────────────────────────────────────────────
        // Barometer gürültüsü: tipik MEMS baro için σ ≈ 0.3–0.5 m
        // R = σ²  →  0.3² = 0.09 m²
        // Gerçek değeri bulmak için: drone'u sabit tutup 1000 örnek kaydet,
        // R = np.var(samples) ile hesapla, sonra buraya yaz.
        Eigen::Matrix<double, 1, 1> R;
        R << 0.09;

        // P0: başlangıç belirsizliği — büyük başlat, filtre hızla yakınsar
        // Küçük başlatırsan filtre ilk saniyelerde yanlış güven duyar
        Eigen::Matrix3d P0 = Eigen::Matrix3d::Zero();
        P0(0, 0) = 5.0;    // z belirsizliği:    ±√5  ≈ ±2.2 m
        P0(1, 1) = 2.0;    // ż belirsizliği:    ±√2  ≈ ±1.4 m/s
        P0(2, 2) = 0.01;   // bias belirsizliği: ±0.1   m/s²
        kf_ = std::make_unique<Drone::Filter::KalmanFilter<3>>(F, B, Q, H, R);

        Eigen::Vector3d x0;
        x0 << initial_baro_height,  // z: ilk baro ölçümü
            0.0,                  // ż: hareketsiz
            0.0;                  // bias: sıfır varsay

        kf_->init(x0, P0);


    }
    
    void HeightEstimator::update_model(double u)
    {
        kf_->prediction(u);
    }

    double HeightEstimator::getHeight() const 
    { 
        return kf_->getState()(0); 
    }

    double HeightEstimator::getVelo() const
    {
        return kf_->getState()(1);
    }

    double HeightEstimator::getBias() const
    {
        return kf_->getState()(2);
    }
    
    void HeightEstimator::update_measurement(double z)
    {
        kf_->update(z);
    }
    
}
