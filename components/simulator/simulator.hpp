#pragma once
#include "car.hpp"
#include "follower.hpp"


typedef void (*point_out)(float x, float y, float direction);

class simulator{
    private:
        bool use_output;
        float step_lengt;
        point_out bb_out;
        point_out trail_out;
        car bb;
        follower trail;
        void step(float move_lenght);

    public:
        simulator(float bbx, float bby, float bbangle, float bbalpha, float followerx, float followery, float step_size);
        void simulate(float lenght);
        void set_output(point_out car,point_out trailer);
};