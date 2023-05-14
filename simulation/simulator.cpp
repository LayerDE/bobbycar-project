#include "simulator.hpp"

void simulator::step(){

}
simulator::simulator(float bbx, float bby, float bbangle, float bbalpha, float followerx, float followery, float step_size){
    bb = car(2,1,bbx,bby,bbangle,bbalpha);
};
void simulator::simulate(float lenght){
    while(lenght-=step_lengt)
        step();
}