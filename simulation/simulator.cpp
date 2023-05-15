#include "simulator.hpp"

void simulator::step(float step_lenght){
    bb.move(step_lenght);

}

simulator::simulator(float bbx, float bby, float bbangle, float bbalpha, float followerx, float followery, float step_size) : bb(2,1,bbx,bby,bbangle,bbalpha),trail(&bb,1,0){
};

void simulator::simulate(float lenght){
    while(lenght-=step_lengt > 0){
        step(step_lengt);
        if(use_output){
            bb_out(bb.x,bb.y,bb.direction);
            trail_out(trail.x, trail.y, trail.direction);
        }
    }
}