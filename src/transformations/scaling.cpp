#include "scaling.h"

scaling::scaling(int scaling_id, float scale_x, float scale_y, float scale_z) {
    this->scaling_id = scaling_id;
    this->scale_x = scale_x;
    this->scale_y = scale_y;
    this->scale_z = scale_z;
}

Eigen::Matrix4f scaling::get_scaling_matrix(){

    Eigen::Matrix4f result;
    result << scale_x, 0.0, 0.0, 0.0,
                0.0, scale_y, 0.0, 0.0,
                0.0, 0.0, scale_z, 0.0,
                0.0, 0.0, 0.0, 1.0;


    return result;

}