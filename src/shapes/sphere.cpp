#include "sphere.h"
#include "../math/math.h"
#include "../transformations/rotation.h"
#include "../transformations/scaling.h"
#include "../transformations/translation.h"
#include <iostream>

sphere::sphere(int id,
               int material_id,
               const std::vector<parser::Material> &materials,
               std::vector<int> &texture_ids,
               int center_id,
               float radius,
               const std::vector<parser::Vec3f> &vertex_data,
               const std::vector<parser::Transformation> &transformations,
               const std::vector<parser::Translation> &translations,
               const std::vector<parser::Rotation> &rotations,
               const std::vector<parser::Scaling> &scalings,
               const std::vector<parser::Composite> &composites,
               parser::Vec3f &radiance,
               bool is_light,
               bool is_cloud): shape(id, material_id, materials, texture_ids, is_light, is_cloud,radiance,"sphere") {


    this->center_id = center_id-1;
    this->radius = radius;
    this->theta = 0.0;
    this->phi = 0.0;

    this->center.x = vertex_data[this->center_id].x;
    this->center.y = vertex_data[this->center_id].y;
    this->center.z = vertex_data[this->center_id].z;

    this->texture_ids = texture_ids;

    this->transformation_matrix = calculate_transformation_matrix(transformations, translations, rotations, scalings, composites);

}

sphere::sphere(parser::Vec3f &center, float radius, const std::vector<parser::Transformation> &transformations,
               const std::vector<parser::Translation> &translations,
               const std::vector<parser::Rotation> &rotations,
               const std::vector<parser::Scaling> &scalings,
               const std::vector<parser::Composite> &composites,
               int material_id,
               std::vector<parser::Material> &materials,
               std::vector<int> &texture_ids,
               parser::Vec3f &radiance,
               bool is_light,
               bool is_cloud): shape(0, material_id, materials, texture_ids, is_light, is_cloud,radiance,"sphere"){

    this->radius = radius;
    this->center = center;
    this->theta = 0.0;
    this->phi = 0.0;

    this->texture_ids = texture_ids;

    this->transformation_matrix = calculate_transformation_matrix(transformations, translations, rotations, scalings, composites);


}

parser::Vec3f sphere::get_texture_values(texture &tex, parser::Vec3f &intersection_point){

    parser::Vec3f sub = vector_subtract(intersection_point, this->center);

    float theta = acos(sub.y / this->radius);
    float phi = atan2(sub.z, sub.x);
    this->theta = theta;
    this->phi = phi;

    parser::Vec2f texture_coordinate;

    texture_coordinate.u = ((-phi + M_PI)/(2*M_PI));
    texture_coordinate.v = (theta / M_PI);

    return tex.get_texture_value(texture_coordinate);
}

parser::Vec2f sphere::get_uv(parser::Vec3f &intersected_point){
    parser::Vec3f sub = vector_subtract(intersected_point, this->center);

    float theta = acos(sub.y / this->radius);
    float phi = atan2(sub.z, sub.x);
    this->theta = theta;
    this->phi = phi;

    parser::Vec2f texture_coordinate;

    texture_coordinate.u = ((-phi + M_PI)/(2*M_PI));
    texture_coordinate.v = (theta / M_PI);

    return texture_coordinate;
}

parser::Vec2f sphere::get_phi_theta(parser::Vec3f &intersected_point){
    parser::Vec3f sub = vector_subtract(intersected_point, this->center);

    float theta = acos(sub.y / this->radius);
    float phi = atan2(sub.z, sub.x);
    return parser::Vec2f{phi, theta};
}


int sphere::get_center_id() {
    return center_id;
}


parser::Vec3f sphere::get_center(){
    return center;
}


float sphere::get_radius(){
    return radius;
}

Eigen::Matrix4f sphere::get_transformation_matrix(){
    return transformation_matrix;
}

std::vector<float> sphere::get_min_max() {
    std::vector<float> min_max_points;

    min_max_points.push_back(center.x + radius + 0.00001);
    min_max_points.push_back(center.x - radius - 0.00001);

    min_max_points.push_back(center.y + radius + 0.00001);
    min_max_points.push_back(center.y - radius - 0.00001);

    min_max_points.push_back(center.z + radius + 0.00001);
    min_max_points.push_back(center.z - radius - 0.00001);

    return min_max_points;
}


parser::Vec3f sphere::get_normal(parser::Vec3f &point){

    parser::Vec3f result = vector_divide(vector_subtract(point, center),
                                         get_vector_magnitude(vector_subtract(point, center)));

    return result;
}


float sphere::get_intersection_parameter(const parser::Vec3f &cam_position,
                                         const parser::Vec3f &direction){


    parser::Vec3f normalized_cam_vector = normalize_vector(direction);
    parser::Vec3f com_vector = vector_subtract(cam_position, center);

    float d_c_dot = dot_product(normalized_cam_vector, com_vector);
    float d_dot = dot_product(normalized_cam_vector, normalized_cam_vector);
    float c_dot = dot_product(com_vector, com_vector);

    float discriminant = pow(d_c_dot,2)-(d_dot*(c_dot-pow(radius,2)));

    float parameter = 0.0;
    float parameter2 = 0.0;


    if(discriminant < 0.0000001 && discriminant > -0.0000001){
        parameter = ((-1* d_c_dot) + sqrt(discriminant)) / (d_dot);

        if(parameter > 0){
            return parameter;
        }else{
            return 0.0;
        }

    }else if(discriminant > 0.0000001){

        parameter = ((-1* d_c_dot) + sqrt(discriminant)) / (d_dot);
        parameter2 = ((-1* d_c_dot) - sqrt(discriminant)) / (d_dot);

        if(parameter < 0 && parameter2 > 0){
            return parameter2;
        }else if(parameter > 0 && parameter2 < 0){
            return parameter;
        }else if(parameter < 0 && parameter2 < 0){
            return 0.0;
        }else{
            if(parameter> parameter2){
                return parameter2;

            }else{
                return parameter;
            }

        }

    }else{
        return 0.0;
    }
}

float sphere::get_cos_theta_max(parser::Vec3f &intersected_point) {

    // Transform intersected point to local space
    Eigen::Matrix4f inverse_transformation_matrix = transformation_matrix.inverse();

    Eigen::Vector4f int_point_4;
    int_point_4 << intersected_point.x, intersected_point.y, intersected_point.z, 1.0;
    int_point_4 = inverse_transformation_matrix * int_point_4;

    // Normalize homogeneous coordinate
    if (std::fabs(int_point_4(3)) > 1e-6f) {
        int_point_4 = int_point_4 / int_point_4(3);
    }

    // Transformed point in local space
    parser::Vec3f intersected_point_transformed{
            int_point_4.x(), int_point_4.y(), int_point_4.z()
    };

    // Compute vector to sphere center
    parser::Vec3f center_vector = vector_subtract(center, intersected_point_transformed);
    float dist = get_vector_magnitude(center_vector);

    // Compute sin(theta_max) with safeguards
    float sin_max = std::fmin(radius / (dist + 1e-6f), 1.0f);

    // Compute cos(theta_max)
    float cos_max = sqrt(1.0f - sin_max * sin_max);

    return cos_max;
}

parser::Vec3f sphere::get_sphere_object_light_ray(parser::Vec3f &intersected_point,
                                                  float rand1, float rand2){


    Eigen::Matrix4f inverse_transformation_matrix = transformation_matrix.inverse();

    Eigen::Vector4f int_point_4;
    int_point_4 << intersected_point.x, intersected_point.y,intersected_point.z, 1.0;
    int_point_4 = inverse_transformation_matrix*int_point_4;

    if(int_point_4(3) != 1.0){
        int_point_4 = int_point_4/int_point_4(3);
    }

    parser::Vec3f intersected_point_transformed{int_point_4.x(), int_point_4.y(), int_point_4.z()};


    parser::Vec3f center_vector = vector_subtract(center, intersected_point_transformed);


    float dist = get_vector_magnitude(center_vector);
    float sin_max = radius / (dist + 1e-2);

    float cos_max = sqrt(std::fmax( 0.0f,1 - pow(sin_max, 2)));

    float phi = 2.0f * M_PI * rand1;
    float theta = acos( 1 - rand2 + rand2*cos_max);

    center_vector = normalize_vector(center_vector);

    parser::Vec3f orth_base_min = center_vector;

    if(fabs(center_vector.x) <= fabs(center_vector.y) && fabs(center_vector.x) <= fabs(center_vector.z)){
        orth_base_min.x = 1.0;
    }else if(fabs(center_vector.y) <= fabs(center_vector.z) && fabs(center_vector.y) <= fabs(center_vector.x)){
        orth_base_min.y = 1.0;
    }else if(fabs(center_vector.z) <= fabs(center_vector.y) && fabs(center_vector.z) <= fabs(center_vector.x)){
        orth_base_min.z = 1.0;
    }

    orth_base_min = normalize_vector(orth_base_min);

    parser::Vec3f u_vec = normalize_vector(cross_product(center_vector, orth_base_min));
    parser::Vec3f v_vec = normalize_vector(cross_product(center_vector, u_vec));

    parser::Vec3f w_elem = vector_multiply(center_vector, cos(theta));
    parser::Vec3f v_elem = vector_multiply(v_vec, sin(theta) * sin(phi));
    parser::Vec3f u_elem = vector_multiply(u_vec, sin(theta) * cos(phi));

    parser::Vec3f final_ray = normalize_vector(vector_add(vector_add(w_elem, v_elem), u_elem));

    float intersection_t = get_intersection_parameter(intersected_point_transformed, final_ray);
    parser::Vec3f point = vector_add(intersected_point_transformed,
                                           vector_multiply(final_ray, intersection_t));



    Eigen::Vector4f point_4;
    point_4 << point.x, point.y,point.z, 1.0;
    point_4 = transformation_matrix*point_4;

    if(point_4(3) != 1.0){
        point_4 = point_4/point_4(3);
    }

    parser::Vec3f final_point{point_4.x(), point_4.y(), point_4.z()};


    return final_point;
}



std::vector<parser::Transformation> sphere::get_transformations(){
    return transformations;
}