#ifndef HW1_NEW_SHAPE_H
#define HW1_NEW_SHAPE_H

#include "../parser/parser.h"
#include "Eigen/Dense"
#include "../textures/texture.h"

class shape {

    private:
        int id;
        int material_id;
        parser::Vec3f ambient_reflectance{};
        parser::Vec3f diffuse_reflectance{};
        parser::Vec3f specular_reflectance{};
        float phong_exponent;
        bool is_mirror;
        parser::Vec3f mirror_coeff{};
        bool is_conductor;
        bool is_dielectric;
        parser::Vec3f absorption_coefficient{};
        float refraction_index;
        float absorption_index;
        float roughness;
        std::string shape_type;
        parser::Vec3f center;
        Eigen::Matrix4f transformation_matrix;
        parser::Material material;
        bool is_light;
        bool is_cloud;
        parser::Vec3f radiance;

    public:
        std::vector<int> texture_ids;
        shape(int id, int material_id, const std::vector<parser::Material> &materials,
              std::vector<int> &texture_ids, bool is_light,bool is_cloud,
              parser::Vec3f &radiance, const std::string &shape_type);
        shape(int id, int material_id, const std::vector<parser::Material> &materials,
              const std::vector<int> &texture_ids, bool is_light,bool is_cloud,
              parser::Vec3f &radiance, const std::string &shape_type);
        shape(const std::string &shape_type);
        parser::Vec3f get_ambient_reflectance();
        parser::Vec3f get_diffuse_reflectance();
        parser::Vec3f get_specular_reflectance();
        float get_phong_exponent() const;
        parser::Vec3f get_mirror_coeff();
        int get_type() const;

        virtual bool get_is_light();
        virtual bool get_is_cloud();

        virtual parser::Vec3f get_radiance();

        virtual parser::Vec3f get_normal(parser::Vec3f &point);
        virtual float get_intersection_parameter(const parser::Vec3f &cam_position,
                                                 const parser::Vec3f &direction);
        virtual parser::Vec3f get_center();
        virtual std::vector<float> get_min_max();
        virtual int get_shape_inside_shape_type();
        virtual int get_index_in_tlas();

        parser::Vec3f get_absorption_coefficient();
        virtual void apply_transformation(Eigen::Matrix4f transformation_matrix);
        float get_refraction_index() const;
        float get_absorption_index() const;
        float get_roughness();
        std::string get_shape_type();

        bool get_mirror() const;
        bool get_dielectric() const;
        bool get_conductor() const;

        virtual Eigen::Matrix4f get_transformation_matrix();
        virtual Eigen::Matrix4f get_blur_matrix();
        virtual int get_instance_index();
        virtual parser::Material get_material_struct();

        virtual parser::Vec3f get_texture_values(texture &tex, parser::Vec3f &intersection_point);

        virtual std::vector<parser::Vec3f> get_corner_world_coordinates();
        virtual std::vector<parser::Vec2f> get_corner_tex_coordinates();
        virtual parser::Vec2f get_uv(parser::Vec3f &intersected_point);
        virtual float get_radius();
        virtual parser::Vec2f get_phi_theta(parser::Vec3f &intersected_point);

        virtual std::vector<int> get_tex_ids();
};


#endif
