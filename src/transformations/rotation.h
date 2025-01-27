#ifndef ADVANCED_RAY_TRACER_ROTATION_H
#define ADVANCED_RAY_TRACER_ROTATION_H
#include "Eigen/Dense"


class rotation {
public:
    int rotationId;
    float angle, rotation_x, rotation_y, rotation_z;

    rotation(int rotationId, float angle, float x, float y, float z);
    Eigen::Matrix4f get_rotation_matrix();
};


#endif
