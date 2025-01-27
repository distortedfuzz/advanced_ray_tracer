#ifndef ADVANCED_RAY_TRACER_COMPOSITE_H
#define ADVANCED_RAY_TRACER_COMPOSITE_H
#include "Eigen/Dense"

class composite {
public:

    Eigen::Matrix4f composite_matrix;
    int compositeId;

    composite(int compositeId, std::vector<float> row_major_elements);
    Eigen::Matrix4f get_composite_matrix();
};


#endif
