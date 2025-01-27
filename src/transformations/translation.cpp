#include "translation.h"

translation::translation(int translation_id, float translation_x, float translation_y, float translation_z) {

    this->translation_id = translation_id;
    this->translation_x = translation_x;
    this->translation_y = translation_y;
    this->translation_z = translation_z;

}

Eigen::Matrix4f translation::get_translation_matrix() {

    Eigen::Matrix4f result;
    result << 1.0, 0.0, 0.0, translation_x,
            0.0, 1.0, 0.0, translation_y,
            0.0, 0.0, 1.0, translation_z,
            0.0, 0.0, 0.0, 1.0;


    return result;

}