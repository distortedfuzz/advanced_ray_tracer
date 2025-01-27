#include "area_light.h"
#include "../math/math.h"
#include <random>
#include <cmath>
#include <iostream>

area_light::area_light(int id, float size, parser::Vec3f position, parser::Vec3f normal, parser::Vec3f radiance){

    this->id = id;
    this->size = size;
    this->position = position;
    this->normal = normalize_vector(normal);
    this->radiance = radiance;

    int normal_min_side = -1;

    if(fabs(normal.x) < fabs(normal.y)){
        if(fabs(normal.z) < fabs(normal.x)){
            normal_min_side = 2;
        }else{
            normal_min_side = 0;
        }
    }else{
        if(fabs(normal.z) < fabs(normal.y)){
            normal_min_side = 2;
        }else{
            normal_min_side = 1;
        }
    }

    parser::Vec3f holder_vec{normal.x, normal.y, normal.z};

    if(normal_min_side == 0){
        holder_vec.x = 1.0;
    }else if(normal_min_side == 1){
        holder_vec.y = 1.0;
    }else{
        holder_vec.z = 1.0;
    }

    holder_vec = normalize_vector(holder_vec);

    this->u_vec = normalize_vector(cross_product(holder_vec, normal));
    this->v_vec = normalize_vector(cross_product(normal, this->u_vec));

}


parser::Vec3f area_light::generate_sample_point(std::mt19937 &gRandomGenerator){

    std::uniform_real_distribution<float> random_values(-0.5, 0.5);


    float rand_u = random_values(gRandomGenerator);
    float rand_v = random_values(gRandomGenerator);

    parser::Vec3f sample_point = vector_add(this->position,
                                            vector_add(vector_multiply(u_vec, rand_u*this->size),
                                                       vector_multiply(v_vec, rand_v* this->size)));


    return sample_point;
}



parser::Vec3f area_light::get_radiance(parser::Vec3f &start_point, parser::Vec3f &area_sample_point){

    parser::Vec3f light_direction_vector = vector_subtract(start_point, area_sample_point);
    parser::Vec3f normal_to_use = normal;

    if(dot_product(light_direction_vector, this->normal) <0){
        normal_to_use = vector_multiply(normal,-1);
    }

    light_direction_vector = normalize_vector(light_direction_vector);
    float cos_a = dot_product(normal_to_use, light_direction_vector);

    //i do the square distance division step in specular and diffusion calculations themselves****important
    parser::Vec3f result = vector_multiply(this->radiance, (cos_a * size * size));


    return result;
}


float area_light::get_size() {
    return size;
}

parser::Vec3f area_light::get_u(){
    return u_vec;
}

parser::Vec3f area_light::get_v(){
    return v_vec;
}