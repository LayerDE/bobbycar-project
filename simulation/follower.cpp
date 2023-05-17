#include "position.hpp"
#include "car.hpp"
#include "follower.hpp"


follower::follower(car* bbc, float len, float hbeta)
        : position(bbc->x,bbc->y,bbc->direction), last_car_pos(bbc->x,bbc->y,bbc->direction){

    lenght = len;
    connected_car = bbc;
    move_straight(-bbc->get_r2h(),0);
    direction+=hbeta;
    move_straight(-lenght,0);
}


bool follower::check_connection(){

}
void follower::set_angle(){

}
void follower::move(float lenght){
    last_car_pos.x = connected_car->x;
    last_car_pos.y = connected_car->y;
    last_car_pos.direction = connected_car->direction;
}

float follower::beta(){
    return direction - connected_car->direction;
}