#include "rotation.h"
#include "../parser/parser.h"
#include "../math/math.h"
#include <cmath>
#include "Eigen/Dense"

rotation::rotation(int rotationId, float angle, float rotation_x, float rotation_y, float rotation_z) {
    this->rotationId = rotationId;
    this->angle = angle;
    this->rotation_x = rotation_x;
    this->rotation_y = rotation_y;
    this->rotation_z = rotation_z;

}

Eigen::Matrix4f rotation::get_rotation_matrix() {
    //ensure axis of rotation is normalized and turned to u_axis
    parser::Vec3f axis_of_rotation;
    axis_of_rotation.x = rotation_x;
    axis_of_rotation.y = rotation_y;
    axis_of_rotation.z = rotation_z;

    parser::Vec3f u_axis = normalize_vector(axis_of_rotation);

    //if axis of rotation already x, y or z we do not need to take the steps of an arbitrary axis
    angle = (angle * M_PI) / 180.0;

    if(u_axis.x == 1){

        Eigen::Matrix4f matrix;
        matrix << 1.0, 0.0, 0.0 ,0.0,
                  0.0, cos(angle), -sin(angle), 0.0,
                  0.0, sin(angle), cos(angle), 0.0,
                  0.0, 0.0, 0.0, 1.0;

        return matrix;

    }else if(u_axis.y == 1){

        Eigen::Matrix4f matrix;
        matrix << cos(angle), 0.0, sin(angle) ,0.0,
                0.0, 1.0, 0.0, 0.0,
                -sin(angle), 0.0, cos(angle), 0.0,
                0.0, 0.0, 0.0, 1.0;

        return matrix;

    }else if(u_axis.z == 1){

        Eigen::Matrix4f matrix;
        matrix << cos(angle), -sin(angle), 0.0 ,0.0,
                sin(angle), cos(angle), 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

        return matrix;

    }

    //continue if it is not x, y or z
    //get the smallest component of u_axis

    std::vector<float> abs_values;

    abs_values.push_back(fabs(rotation_x));
    abs_values.push_back(fabs(rotation_y));
    abs_values.push_back(fabs(rotation_z));

    float smallest_element = 0.0;
    int smallest_index = 0;

    for(int i = 0; i < 3; i++){
        if(i == 0){
            smallest_element = abs_values[0];
        }else{
            if(abs_values[i] < smallest_element){
                smallest_element = abs_values[i];
                smallest_index = i;
            }
        }
    }

    //form v_axis, set smallest of u to 0 and change places of others and negate one
    parser::Vec3f v_axis;

    if(smallest_index == 0){
        v_axis.x = 0.0;

        double hold_y = v_axis.y;
        v_axis.y = -v_axis.z;
        v_axis.z = hold_y;

    }else if(smallest_index == 1){
        v_axis.y = 0.0;

        double hold_x = v_axis.x;
        v_axis.x = -v_axis.z;
        v_axis.z = hold_x;
    }else{
        v_axis.z = 0.0;

        double hold_x = v_axis.x;
        v_axis.x = -v_axis.y;
        v_axis.y = hold_x;
    }

    v_axis = normalize_vector(v_axis);

    //w_axis is just the cross product of u and v

    parser::Vec3f w_axis = cross_product(u_axis, v_axis);

    //rotation matrix for onb to fit xyz


    Eigen::Matrix4f rotation_matrix;
    rotation_matrix << u_axis.x, u_axis.y, u_axis.z ,0.0,
                        v_axis.x, v_axis.y, v_axis.z, 0.0,
                        w_axis.x, w_axis.y, w_axis.z, 0.0,
                        0.0, 0.0, 0.0, 1.0;

    // matrix for reversing the onb to original location after main rotation


    Eigen::Matrix4f reverse_rotation_matrix;
    reverse_rotation_matrix << u_axis.x, v_axis.x,  w_axis.x ,0.0,
                                u_axis.y, v_axis.y, w_axis.y, 0.0,
                                u_axis.z, v_axis.z, w_axis.z, 0.0,
                                0.0, 0.0, 0.0, 1.0;

    //main rotation matrix around x

    Eigen::Matrix4f main_rotation_matrix;
    main_rotation_matrix << 1.0, 0.0,  0.0 ,0.0,
            0.0, cos(angle), -sin(angle), 0.0,
            0.0, sin(angle), cos(angle), 0.0,
            0.0, 0.0, 0.0, 1.0;

    //compute the multiplication and return
    Eigen::Matrix4f final_matrix = reverse_rotation_matrix* main_rotation_matrix;
    final_matrix = final_matrix* rotation_matrix;


    return final_matrix;
}