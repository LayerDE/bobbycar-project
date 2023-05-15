#include "position.hpp"
#include "car.hpp"
#include "follower.hpp"

follower::follower(car* bbc, float len, float hbeta) : position(bbc->x,bbc->y,bbc->direction){
    beta = hbeta;
    lenght = len;
    move_straight(-bbc->get_r2h(),0);
    direction+=beta;
    move_straight(-lenght,0);
}

follower::follower(float len, float hx, float hy, car* bbc) : position(hx,hy,0){
    lenght = len;
}