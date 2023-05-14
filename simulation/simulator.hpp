#pragma once
#include "car.hpp"
#include "follower.hpp"

class simulator{
    private:
        float step_lengt;
        car bb;
        follower trail;
        void step();
    public:
        simulator(float bbx, float bby, float bbangle, float bbalpha, float followerx, float followery, float step_size);
        void simulate(float lenght);
}