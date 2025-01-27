#ifndef ADVANCED_RAY_TRACER_ENVIRONMENT_LIGHT_H
#define ADVANCED_RAY_TRACER_ENVIRONMENT_LIGHT_H


#include "../parser/parser.h"

class environment_light {

public:
    int id;
    /*
     * 0 = latlong
     * 1 = probe
     */
    int type;
    int image_id;
    int width, height;
    float* image_data_exr;

    environment_light(int id, int type, int image_id, std::vector<parser::Image> &images);

    parser::Vec3f get_environment_luminance(parser::Vec3f &sample);
    parser::Vec2f get_uv(parser::Vec3f &sample);

    parser::Vec2f get_uv_latlong(parser::Vec3f &sample);
    parser::Vec2f get_uv_probe(parser::Vec3f &sample);
};




#endif
