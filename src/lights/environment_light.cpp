#include "environment_light.h"
#include <iostream>

environment_light::environment_light(int id, int type, int image_id, std::vector<parser::Image> &images){

    this->id = id;
    this->type = type;
    this->image_id = image_id;
    this->image_data_exr = images[image_id-1].exr_data;
    this->width = images[image_id-1].width;
    this->height = images[image_id-1].height;

}


parser::Vec3f environment_light::get_environment_luminance(parser::Vec3f &sample){


    parser::Vec2f texture_coordinates = get_uv(sample);
    //std::cout<<texture_coordinates.u<<" "<<texture_coordinates.v<<std::endl;

    int i = 0;
    int j = 0;

    if(type == 1){
        i = texture_coordinates.u * width;
        j = texture_coordinates.v * height;

    }else if(type == 0){
        i = (texture_coordinates.u / 2.0f) * width;
        j = texture_coordinates.v * height;

    }

    float r = image_data_exr[(j * width + i) * 4];
    float g = image_data_exr[(j * width + i) * 4 + 1];
    float b = image_data_exr[(j * width + i) * 4 + 2];

    parser::Vec3f sampled_color{r,g,b};

    return sampled_color;
}


parser::Vec2f environment_light::get_uv(parser::Vec3f &sample){

    if(type == 0){
        return get_uv_latlong(sample);
    }else if(type == 1){
        return get_uv_probe(sample);
    }

}


parser::Vec2f environment_light::get_uv_latlong(parser::Vec3f &sample){

    parser::Vec2f uv{0,0};

    uv.u = 1.0f + (( atan2(sample.x, -sample.z) / M_PI));

    uv.v = acos(sample.y) / M_PI;

    return uv;

}


parser::Vec2f environment_light::get_uv_probe(parser::Vec3f &sample){

    float r = (acos(- sample.z)) /
            (M_PI * sqrt(sample.x*sample.x + sample.y*sample.y));

    parser::Vec2f uv{0,0};

    uv.u = ((r*sample.x) + 1.0f) / 2.0f;

    uv.v = ((-r*sample.y) + 1.0f) / 2.0f;

    return uv;
}