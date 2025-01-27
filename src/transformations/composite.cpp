#include "composite.h"
#include <iostream>

composite::composite(int compositeId, std::vector<float> row_major_elements) {
    this->compositeId = compositeId;
    this->composite_matrix << row_major_elements[0], row_major_elements[1], row_major_elements[2], row_major_elements[3],
                             row_major_elements[4], row_major_elements[5], row_major_elements[6], row_major_elements[7],
                             row_major_elements[8], row_major_elements[9], row_major_elements[10], row_major_elements[11],
                             row_major_elements[12], row_major_elements[13], row_major_elements[14], row_major_elements[15];
}


Eigen::Matrix4f composite::get_composite_matrix(){
    return this->composite_matrix;
}