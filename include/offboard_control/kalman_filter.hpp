#ifndef HPP_KALMAN_FILTER
#define HPP_KALMAN_FILTER

#include <Eigen/Dense>

namespace Drone::Filter
{

// [1] Template parametresi: R state boyutu, C ise F matrisinin sütun sayısı.
//     Kalman filter matematiksel olarak F'nin kare matris olmasını zorunlu kılar:
//     x(k) = F * x(k-1) — bu çarpımın geçerli olabilmesi için F'nin R×R olması gerekir.
//     Yani C her zaman R'ye eşit olmak zorunda. Ayrı bir C parametresi tutmak
//     gereksiz bir serbestlik derecesi bırakıyor ve derleme zamanında yakalanabilecek
//     bir hatayı gizleyebilir.
template <size_t N>
class KalmanFilter
{
    // Sık kullanılan Eigen tiplerini takma adlarla tanımlıyoruz.
    // Her yerde Matrix<double, N, N> yazmak hem hatalara kapı aralar
    // hem de okunabilirliği düşürür.
    using StateVec  = Eigen::Vector<double, N>;
    using StateMat  = Eigen::Matrix<double, N, N>;
    using ObsRowVec = Eigen::Matrix<double, 1, N>;
    using Scalar1x1 = Eigen::Matrix<double, 1, 1>;
    using ColVecN   = Eigen::Matrix<double, N, 1>;

public:

    // [2] Constructor parametreleri const& ile alınıyordu — doğru niyet,
    //     ama member initialization list'te doğrudan kopyalama yapılıyordu.
    //     Parametreleri by-value alıp std::move ile taşırsak, çağıran taraf
    //     rvalue geçtiğinde ekstra kopya tamamen ortadan kalkar.
    //     (R_ constructor'a taşındı — önceki versiyonda constructor'da yoktu,
    //      init edilmemiş R_ ile update() çağrısı UB üretirdi.)
    KalmanFilter(
        StateMat   F,
        ColVecN    B,
        StateMat   Q,
        ObsRowVec  H,
        Scalar1x1  R)
        : F_(std::move(F))
        , B_(std::move(B))
        , Q_(std::move(Q))
        , H_(std::move(H))
        , R_(std::move(R))
        , initialized_(false)
    {
    }

    // [3] init() parametreleri önceden by-value alınıyordu — gereksiz kopya.
    //     Sadece okuma amaçlı olduğu için const& yeterli.
    void init(const StateVec& init_x, const StateMat& init_P)
    {
        x_           = init_x;
        P_           = init_P;
        initialized_ = true;
    }

    // [4] prediction() ve update() çağrılmadan önce init() yapılıp
    //     yapılmadığını kontrol etmiyorduk. Başlatılmamış x_ ve P_ ile
    //     çalışmak undefined behavior üretir. initialized_ flag'i eklendi;
    //     debug modunda assert ile korunuyor, release'de sıfır maliyet.
    void prediction(double u)
    {   
        assert(initialized_ && "KalmanFilter::prediction() called before init()");

        x_ = F_ * x_ + B_ * u;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(double z)
    {
        assert(initialized_ && "KalmanFilter::update() called before init()");

        // [5] (H_ * x_) ifadesi Matrix<1,1> döndürür — skaler değere
        //     ulaşmak için .value() gerekir. Önceki kodda bu implicit
        //     dönüşüme bırakılmıştı, bazı Eigen versiyonlarında derleme
        //     uyarısı veya hata üretebilir.
        const double y = z - (H_ * x_).value();

        // [6] S Matrix<1,1> olarak tutuluyordu ve üzerinde .inverse()
        //     çağrılıyordu. 1×1 matris üzerinde .inverse() gereksiz yere
        //     LU decomposition başlatır. S'i double olarak tutup
        //     scalar bölme yapmak hem daha hızlı hem de daha okunabilir.
        const double S = (H_ * P_ * H_.transpose()).value() + R_.value();

        K_ = P_ * H_.transpose() / S;
        x_ = x_ + K_ * y;

        // [7] Standart kovaryans güncellemesi: P = (I - KH) * P
        //     Numerik olarak kararsız — floating point hataları zamanla
        //     P'nin simetrisini ve pozitif tanımlılığını bozar.
        //     Joseph form: P = (I-KH)*P*(I-KH)^T + K*R*K^T
        //     Her zaman simetrik ve pozitif yarı-tanımlı sonuç garanti eder.
        //     Uzun süreli uçuşlarda bu kritik fark yaratır.
        const StateMat IKH = StateMat::Identity() - K_ * H_;
        P_ = IKH * P_ * IKH.transpose() + K_ * R_.value() * K_.transpose();
    }

    // [8] getState() önceki versiyonda by-value dönüyordu — küçük sabit
    //     boyutlu Eigen vektörleri için kabul edilebilir ama const& daha
    //     açık bir niyet ifadesi ve gereksiz kopyayı önler.
    //     getCovariance() ve isInitialized() eklendi — dışarıdan filtre
    //     durumunu izlemek (NIS hesabı, debug) için gerekli.
    const StateVec& getState()      const { return x_; }
    const StateMat& getCovariance() const { return P_; }
    bool            isInitialized() const { return initialized_; }

private:

    StateVec  x_;     // State vector
    StateMat  F_;     // State Transition Matrix
    ColVecN   B_;     // Control Input Matrix
                      //w(k) Process Noise 
                      //      IMU'nun gürültüsünü ve modelimizin mükemmel olmadığını temsil eder.
                      //      Kovaryansı: Q matrix
    StateMat  Q_;     // Process Noise Covariance, Örnek: 
                      //      Height estimation: IMU'ma ve fizik modelime ne kadar güveniyorum
    ObsRowVec H_;     // Observation Matrix
    Scalar1x1 R_;     // Measurement Noise, Örnek: 
                      //      Height estimation: Barometreme ne kadar güveniyorum
    StateMat  P_;     // Error Covariance: Tahminimize ne kadar güvendiğimizin sayısal ifadesi.
    ColVecN   K_;     // Kalman Gain

    bool initialized_;
};

} // namespace Drone::px4_offboard

#endif // HPP_KALMAN_FILTER