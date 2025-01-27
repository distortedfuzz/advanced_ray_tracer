#ifndef ADVANCED_RAY_TRACER_AREA_LIGHT_H
#define ADVANCED_RAY_TRACER_AREA_LIGHT_H
#include "../parser/parser.h"
#include "Eigen/Dense"
#include <random>

class area_light {
    int id;
    float size;

    parser::Vec3f radiance;
    parser::Vec3f normal;
    parser::Vec3f u_vec;
    parser::Vec3f v_vec;

public:

    parser::Vec3f position;

    area_light(int id, float size, parser::Vec3f position, parser::Vec3f normal, parser::Vec3f radiance);
    parser::Vec3f generate_sample_point(std::mt19937 &gRandomGenerator);
    parser::Vec3f get_radiance(parser::Vec3f &start_point, parser::Vec3f &area_sample_point);
    float get_size();
    parser::Vec3f get_u();
    parser::Vec3f get_v();

};


#endif
