#include "shape.h"


shape::shape(int id, int material_id, const std::vector<parser::Material> &materials,
             std::vector<int> &texture_ids, bool is_light, bool is_cloud,
             parser::Vec3f &radiance, const std::string &shape_type) {

    this->id = id;
    this->material_id = material_id-1;
    this->is_light = is_light;
    this->is_cloud = is_cloud;

    this->radiance = radiance;

    for(auto tex: texture_ids){
        this->texture_ids.push_back(tex-1);
    }
    this->material = materials[material_id-1];
    if(materials.size()> 0){
        this->ambient_reflectance = materials[this->material_id].ambient;
        this->diffuse_reflectance = materials[this->material_id].diffuse;
        this->specular_reflectance = materials[this->material_id].specular;
        this->phong_exponent = materials[this->material_id].phong_exponent;
        this->is_mirror = materials[this->material_id].is_mirror;
        this->is_conductor = materials[this->material_id].is_conductor;
        this->is_dielectric = materials[this->material_id].is_dielectric;
        this->roughness = materials[this->material_id].roughness;
        this->mirror_coeff = materials[this->material_id].mirror;
        this->absorption_coefficient = materials[this->material_id].absorption_coefficient;
        this->refraction_index = materials[this->material_id].refraction_index;
        this->absorption_index = materials[this->material_id].absorption_index;
    }

    this->shape_type = shape_type;
    this->transformation_matrix = Eigen::Matrix4f::Identity();
}

shape::shape(int id, int material_id, const std::vector<parser::Material> &materials,
             const std::vector<int> &texture_ids, bool is_light, bool is_cloud,
             parser::Vec3f &radiance, const std::string &shape_type) {

    this->id = id;
    this->material_id = material_id-1;
    this->material = materials[material_id-1];

    this->is_light = is_light;
    this->is_cloud = is_cloud;
    this->radiance = radiance;

    for(auto tex: texture_ids){
        this->texture_ids.push_back(tex-1);
    }
    if(materials.size()> 0){
        this->ambient_reflectance = materials[this->material_id].ambient;
        this->diffuse_reflectance = materials[this->material_id].diffuse;
        this->specular_reflectance = materials[this->material_id].specular;
        this->phong_exponent = materials[this->material_id].phong_exponent;
        this->is_mirror = materials[this->material_id].is_mirror;
        this->is_conductor = materials[this->material_id].is_conductor;
        this->is_dielectric = materials[this->material_id].is_dielectric;
        this->roughness = materials[this->material_id].roughness;
        this->mirror_coeff = materials[this->material_id].mirror;
        this->absorption_coefficient = materials[this->material_id].absorption_coefficient;
        this->refraction_index = materials[this->material_id].refraction_index;
        this->absorption_index = materials[this->material_id].absorption_index;
    }

    this->shape_type = shape_type;
    this->transformation_matrix = Eigen::Matrix4f::Identity();
}

shape::shape(const std::string &shape_type) {

    this->shape_type = shape_type;
}


parser::Vec3f shape::get_ambient_reflectance(){
    return ambient_reflectance;
}

parser::Vec3f shape::get_diffuse_reflectance(){
    return diffuse_reflectance;
}

parser::Vec3f shape::get_specular_reflectance(){
    return specular_reflectance;
}

float shape::get_phong_exponent() const{
    return phong_exponent;
}

parser::Vec3f shape::get_mirror_coeff() {
    return mirror_coeff;
}

float shape::get_roughness(){
    return this->roughness;
}

/* 0: mirror
 * 1: conductor
 * 2: dielectric
 * 3: unkknown*/
int shape::get_type() const{
    if(is_mirror){
        return 0;
    }else if(is_conductor){
        return 1;
    }else if(is_dielectric){
        return 2;
    }else{
        return 3;
    }
}

bool shape::get_is_light()
{
    return is_light;
}

bool shape::get_is_cloud()
{
    return is_cloud;
}

parser::Vec3f shape::get_radiance(){
    return radiance;
}

parser::Vec3f shape::get_center(){
    parser::Vec3f new_center{0,0,0};
    return new_center;
}

std::vector<float> shape::get_min_max(){
    std::vector<float> min_max_points;

    for(int i = 0; i < 6; i++){
        min_max_points.push_back(0.0);
    }
    return min_max_points;
}


parser::Vec3f shape::get_normal(parser::Vec3f &point) {
    parser::Vec3f new_normal{0,0,0};
    return new_normal;
}

float shape::get_intersection_parameter(const parser::Vec3f &cam_position,
                                        const parser::Vec3f &direction) {
    return 0.0;
}

parser::Vec3f shape::get_absorption_coefficient() {
    return absorption_coefficient;

}

float shape::get_refraction_index() const{
    return refraction_index;

}

float shape::get_absorption_index() const{
    return absorption_index;

}

std::string shape::get_shape_type(){
    return shape_type;
}

bool shape::get_mirror() const{
    return is_mirror;
}
bool shape::get_dielectric() const{
    return is_dielectric;
}
bool shape::get_conductor() const{
    return is_conductor;
}

void shape::apply_transformation(Eigen::Matrix4f transformation_matrix){

}


Eigen::Matrix4f shape::get_transformation_matrix(){
    return transformation_matrix;
}

int shape::get_instance_index(){
    return 0;
}
Eigen::Matrix4f shape::get_blur_matrix(){

}
int shape::get_shape_inside_shape_type(){
    return -1;
}

int shape::get_index_in_tlas(){
    return 0;
}

parser::Material shape::get_material_struct(){
    return material;
}

parser::Vec3f shape::get_texture_values(texture &tex, parser::Vec3f &intersection_point){

    return parser::Vec3f{0,0,0};

}

std::vector<parser::Vec3f> shape::get_corner_world_coordinates(){

    std::vector<parser::Vec3f> res;
    res.push_back(parser::Vec3f{0,0,0});

    return res;

}

std::vector<parser::Vec2f> shape::get_corner_tex_coordinates(){
    std::vector<parser::Vec2f> res;
    res.push_back(parser::Vec2f{0,0});

    return res;
}

parser::Vec2f shape::get_uv(parser::Vec3f &intersected_point){
    return parser::Vec2f{0,0};
}

float shape::get_radius(){
    return 0.0;
}

parser::Vec2f shape::get_phi_theta(parser::Vec3f &intersected_point){
    return parser::Vec2f{0,0};
}

std::vector<int> shape::get_tex_ids(){
    std::vector<int> res;

    return res;
}