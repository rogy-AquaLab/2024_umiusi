#ifndef POWER_MAP_CONTAINER_HPP
#define POWER_MAP_CONTAINER_HPP

namespace power_map {

template<class T> class Container {
private:
    T _bldc_center;
    T _bldc_positive_radius;
    T _bldc_negative_radius;
    T _servo_min;
    T _servo_max;

public:
    Container() = default;

    auto bldc_center() const -> const T& {
        return this->_bldc_center;
    }

    auto bldc_center(T&& v) -> Container& {
        this->_bldc_center = v;
        return *this;
    }

    auto bldc_positive_radius() const -> const T& {
        return this->_bldc_positive_radius;
    }

    auto bldc_positive_radius(T&& v) -> Container& {
        this->_bldc_positive_radius = v;
        return *this;
    }

    auto bldc_negative_radius() const -> const T& {
        return this->_bldc_negative_radius;
    }

    auto bldc_negative_radius(T&& v) -> Container& {
        this->_bldc_negative_radius = v;
        return *this;
    }

    auto servo_min() const -> const T& {
        return this->_servo_min;
    }

    auto servo_min(T&& v) -> Container& {
        this->_servo_min = v;
        return *this;
    }

    auto servo_max() const -> const T& {
        return this->_servo_max;
    }

    auto servo_max(T&& v) -> Container& {
        this->_servo_max = v;
        return *this;
    }
};

}

#endif // POWER_MAP_CONTAINER
