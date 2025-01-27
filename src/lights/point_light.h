#ifndef HW1_NEW_POINT_LIGHT_H
#define HW1_NEW_POINT_LIGHT_H
#include "../parser/parser.h"
#include "Eigen/Dense"

class point_light {
    private:
        int id;
        parser::Vec3f position;
        parser::Vec3f intensity;
        Eigen::Matrix4f transformation_matrix;
        std::vector<parser::Transformation> transformations;

    public:
        point_light(int id, parser::Vec3f position, parser::Vec3f intensity,
                    const std::vector<parser::Transformation> &transformations,
                    const std::vector<parser::Translation> &translations,
                    const std::vector<parser::Rotation> &rotations,
                    const std::vector<parser::Scaling> &scalings,
                    const std::vector<parser::Composite> &composites);

        int get_id();

        parser::Vec3f get_position();
        parser::Vec3f get_intensity();
};


#endif