#ifndef ADVANCED_RAY_TRACER_SCALING_H
#define ADVANCED_RAY_TRACER_SCALING_H
#include "Eigen/Dense"

class scaling {
public:
    int scaling_id;
    float scale_x, scale_y, scale_z;

    scaling(int scaling_id, float scale_x, float scale_y, float scale_z);

    Eigen::Matrix4f get_scaling_matrix();
};


#endif
