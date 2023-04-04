#pragma once

class follower{
    public:
        follower(int to_car_axle, int to_follower_axle);
        int get_speed();
        float get_steering();
        void calculate(int des_speed, float des_steering);
    private:
        void math_model();
        int lenght;
        int axle;

};