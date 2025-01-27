#include "directional_light.h"

directional_light::directional_light(int id, parser::Vec3f direction, parser::Vec3f radiance){

    this->id = id;
    this->direction = normalize_vector(direction);
    this->radiance = radiance;

}

int directional_light::get_id(){
    return this->id;
}

parser::Vec3f directional_light::get_direction(){
    return this->direction;
}

parser::Vec3f directional_light::get_radiance(){
    return this->radiance;
}