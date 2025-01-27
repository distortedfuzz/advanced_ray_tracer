#ifndef ADVANCED_RAY_TRACER_DIRECTIONAL_LIGHT_H
#define ADVANCED_RAY_TRACER_DIRECTIONAL_LIGHT_H

#include "../parser/parser.h"
#include "../math/math.h"

class directional_light {
private:
    int id;
    parser::Vec3f direction;
    parser::Vec3f radiance;

public:
    directional_light(int id, parser::Vec3f direction, parser::Vec3f radiance);

    int get_id();

    parser::Vec3f get_direction();
    parser::Vec3f get_radiance();
};


#endif
