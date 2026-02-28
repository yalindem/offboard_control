#ifndef HPP_MEDIAN_FILTER
#define HPP_MEDIAN_FILTER

#include <array>
#include <algorithm> // std::sort ve std::nth_element için gerekli

namespace Drone::px4_offboard
{
    template<size_t N>
    class FastMedianFilter
    {
        private:
            std::array<float, N> buffer_{};
            size_t index_ = 0;
            size_t count_ = 0;

        public:
            float update(float value)
            {
                buffer_[index_] = value;
                index_ = (index_ + 1) % N;
                
                if (count_ < N) count_++;

                // Geçici bir kopya oluştur (Sadece eklenen veri kadar)
                std::array<float, N> temp = buffer_;

                // Tam sıralama yerine sadece orta elemanı bul (Daha hızlı)
                auto mid = temp.begin() + count_ / 2;
                std::nth_element(temp.begin(), mid, temp.begin() + count_);
                
                return *mid;
            }

            void reset() {
                buffer_.fill(0.0f);
                index_ = 0;
                count_ = 0;
            }

            bool isFilled() const { return count_ == N; }
            std::size_t size() const { return N; }
    };
}

#endif