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
