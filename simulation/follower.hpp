#pragma once
#include "car.hpp"
#include "position.hpp"

class follower : public position{
    private:
        car* connected_car;
        float lenght;
        position last_car_pos;
    public:
        float angle;
        follower(float len, float hx, float hy, car* bbc) : position(hx,hy,0){
            
        }
        bool check_connection();
        void set_angle();
        void move(float lenght);
};