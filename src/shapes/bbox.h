#ifndef ADVANCED_RAY_TRACER_BBOX_H
#define ADVANCED_RAY_TRACER_BBOX_H
#include <memory>
#include "../parser/parser.h"
#include "shape.h"
#include "Eigen/Dense"

class bbox : public shape {
private:
    std::vector<parser::Vec3f> min_max_corners;
    bool is_full_object;

public:
    Eigen::Matrix4f transformation_matrix;
    Eigen::Matrix4f motion_blur_matrix;
    std::vector<parser::Vec3f> all_corners;
    int object_type;
    int object_index_in_tlas_vector;
    parser::Material bbox_material;
    std::vector<int> texture_ids;
    bool is_light;
    bool is_cloud;
    parser::Vec3f radiance;

    bbox(std::vector<std::shared_ptr<shape>> &shapes, int start, int end);

    bbox(std::vector<std::shared_ptr<shape>> &shapes, int start, int end, bool is_full_object,
         int type, int index_in_tlas, parser::Material &material, bool is_light, bool is_cloud,
         parser::Vec3f &radiance,std::vector<int> &tex_ids);

    bbox(Eigen::Matrix4f &matrix,std::shared_ptr<bbox> &bbox_start, std::shared_ptr<bbox> &bbox_end,
         bool is_full_object , int type, int index_in_tlas, parser::Material &material, bool is_light,
         bool is_cloud, parser::Vec3f &radiance,std::vector<int> &tex_ids);

    parser::Vec3f get_dimension_differences();
    std::vector<parser::Vec3f> get_corners();
    bool does_intersect(const parser::Vec3f &cam_position,
                                     const parser::Vec3f &direction);
    parser::Vec3f get_center() override;
    std::vector<float> get_min_max() override;

    bool get_is_light() override;
    bool get_is_cloud() override;
    parser::Vec3f get_radiance() override;


    void apply_transformation(Eigen::Matrix4f transformation_matrix) override;
    Eigen::Matrix4f get_transformation_matrix() override;
    Eigen::Matrix4f get_blur_matrix() override;
    int get_shape_inside_shape_type() override;
    int get_index_in_tlas() override;
    parser::Material get_material_struct() override;
    float get_bbox_intersection_parameter(const parser::Vec3f &ray_origin, const parser::Vec3f &ray_direction);

    std::vector<int> get_tex_ids() override;
};


#endif
