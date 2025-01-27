#ifndef ADVANCED_RAY_TRACER_TRANSLATION_H
#define ADVANCED_RAY_TRACER_TRANSLATION_H
#include "Eigen/Dense"

class translation {

public:
    int translation_id;
    float translation_x, translation_y, translation_z;

    translation(int translation_id, float translation_x, float translation_y, float translation_z);

    Eigen::Matrix4f get_translation_matrix();
};


#endif
