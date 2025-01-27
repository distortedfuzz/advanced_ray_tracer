#include "point_light.h"
#include "../math/math.h"

point_light::point_light(int id, parser::Vec3f position, parser::Vec3f intensity,
                         const std::vector<parser::Transformation> &transformations,
                         const std::vector<parser::Translation> &translations,
                         const std::vector<parser::Rotation> &rotations,
                         const std::vector<parser::Scaling> &scalings,
                         const std::vector<parser::Composite> &composites) {

    this->id = id;
    this->position = position;
    this->intensity = intensity;
    this->transformation_matrix = calculate_transformation_matrix(transformations, translations, rotations, scalings, composites);

    Eigen::Vector4f position_hold;
    position_hold << this->position.x, this->position.y,this->position.z, 1.0;
    position_hold = transformation_matrix*position_hold;

    if(position_hold(3) != 1.0){
        position_hold = position_hold/position_hold(3);
    }

    parser::Vec3f pos_tr{position_hold(0), position_hold(1), position_hold(2)};
    this->position = pos_tr;
}


int point_light::get_id() {

    return this->id;
}

parser::Vec3f point_light::get_intensity() {

    return this->intensity;

}

parser::Vec3f point_light::get_position(){

    return this->position;

}