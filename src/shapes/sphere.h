#ifndef HW1_NEW_SPHERE_H
#define HW1_NEW_SPHERE_H

#include "shape.h"
#include "Eigen/Dense"

class sphere : public shape {
    private:
        int center_id;
        parser::Vec3f center;
        float radius;
        Eigen::Matrix4f transformation_matrix;
        std::vector<parser::Transformation> transformations;
        float phi;
        float theta;

    public:
        sphere(int id,
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
               bool is_cloud);

        sphere(parser::Vec3f &center, float radius, const std::vector<parser::Transformation> &transformations,
               const std::vector<parser::Translation> &translations,
               const std::vector<parser::Rotation> &rotations,
               const std::vector<parser::Scaling> &scalings,
               const std::vector<parser::Composite> &composites,
               int material_id,
               std::vector<parser::Material> &materials,
               std::vector<int> &texture_ids,
               parser::Vec3f &radiance,
               bool is_light,
               bool is_cloud);

        int get_center_id();
        parser::Vec3f get_center() override;
        float get_radius() override;
        Eigen::Matrix4f get_transformation_matrix() override;

        parser::Vec3f get_normal(parser::Vec3f &point) override;
        std::vector<float> get_min_max() override;

        float get_intersection_parameter(const parser::Vec3f &cam_position,
                                         const parser::Vec3f &direction) override;

        parser::Vec3f get_texture_values(texture &tex, parser::Vec3f &intersection_point) override;
        std::vector<parser::Transformation> get_transformations();
        parser::Vec2f get_phi_theta(parser::Vec3f &intersected_point) override;
        parser::Vec2f get_uv(parser::Vec3f &intersected_point) override;

        parser::Vec3f get_sphere_object_light_ray(parser::Vec3f &intersected_point,
                                                  float rand1, float rand2);

        float get_cos_theta_max(parser::Vec3f &intersected_point);

};


#endif
