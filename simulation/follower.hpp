#pragma once
#include "car.hpp"
#include "position.hpp"

class follower : public position{
    private:
        car* connected_car;
        float lenght;
        float beta();
        position last_car_pos;
    public:
        float angle;
//        follower(float len, float hx, float hy, car* bbc);
        follower(car* bbc, float len, float hbeta);
        bool check_connection();
        void set_angle();
        void move();
};