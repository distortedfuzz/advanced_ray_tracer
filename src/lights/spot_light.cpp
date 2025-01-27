#include "spot_light.h"

spot_light::spot_light(int id, parser::Vec3f position, parser::Vec3f direction,
                        parser::Vec3f intensity, float coverage_angle, float falloff_angle){

    this->id = id;
    this->position = position;
    this->direction = normalize_vector(direction);
    this->intensity = intensity;
    this->coverage_angle = coverage_angle;
    this->falloff_angle = falloff_angle;

}

int spot_light::get_id(){
    return this->id;
}

parser::Vec3f spot_light::get_position(){
    return this->position;
}

parser::Vec3f spot_light::get_direction(){
    return this->direction;
}

parser::Vec3f spot_light::get_intensity(){
    return this->intensity;
}

float spot_light::get_coverage_angle(){
    return this->coverage_angle;
}

float spot_light::get_falloff_angle(){
    return this->falloff_angle;
}