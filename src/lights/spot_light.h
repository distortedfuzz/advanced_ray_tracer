#ifndef ADVANCED_RAY_TRACER_SPOT_LIGHT_H
#define ADVANCED_RAY_TRACER_SPOT_LIGHT_H

#include "../parser/parser.h"
#include "../math/math.h"


class spot_light {
private:
    int id;
    parser::Vec3f position;
    parser::Vec3f direction;
    parser::Vec3f intensity;
    float coverage_angle;
    float falloff_angle;

public:
    spot_light(int id, parser::Vec3f position, parser::Vec3f direction,
               parser::Vec3f intensity, float coverage_angle, float falloff_angle);

    int get_id();
    parser::Vec3f get_position();
    parser::Vec3f get_direction();
    parser::Vec3f get_intensity();
    float get_coverage_angle();
    float get_falloff_angle();
};


#endif
