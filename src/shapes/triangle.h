#ifndef HW1_NEW_TRIANGLE_H
#define HW1_NEW_TRIANGLE_H

#include "shape.h"
#include "../parser/parser.h"
#include "Eigen/Dense"
#include "../textures/texture.h"

class triangle : public shape{
    private:
        parser::Vec3i indices;
        std::vector<parser::Vec2f> tex_coords;
        std::vector<parser::Vec3f> corners;
        parser::Vec3f normal;
        parser::Vec3f center;
        Eigen::Matrix4f transformation_matrix;

    public:
        std::vector<int> texture_ids;
        triangle(int id,
                 int material_id,
                 std::vector<int> &texture_ids,
                 const std::vector<parser::Material> &materials,
                 const parser::Vec3i &indices,
                 const std::vector<parser::Vec2f> &tex_coord_data,
                 const std::vector<parser::Vec3f> &vertex_data,
                 const std::vector<parser::Transformation> &transformations,
                 const std::vector<parser::Translation> &translations,
                 const std::vector<parser::Rotation> &rotations,
                 const std::vector<parser::Scaling> &scalings,
                 const std::vector<parser::Composite> &composites,
                 int vertex_offset,
                 int texture_offset,
                 parser::Vec3f radiance,
                 bool is_light,
                 bool is_cloud);

        triangle(std::vector<parser::Vec3f> &corners,
                 Eigen::Matrix4f &tr_matrix, int material_id,std::vector<int> &texture_ids,
                 std::vector<parser::Material> &materials,
                 parser::Vec3f radiance,
                 bool is_light,
                 bool is_cloud);


        parser::Vec3i get_indices();
        std::vector<parser::Vec3f> get_corners();
        std::vector<float> get_min_max() override;
        Eigen::Matrix4f get_transformation_matrix() override;

        parser::Vec3f get_normal(parser::Vec3f &point) override;
        parser::Vec3f get_center() override;

        float get_intersection_parameter(const parser::Vec3f &cam_position,
                                         const parser::Vec3f &ray_direction) override;

        parser::Vec3f get_texture_values(texture &tex, parser::Vec3f &intersection_point) override;
        void apply_transformation();

        std::vector<parser::Vec3f> get_corner_world_coordinates() override;
        std::vector<parser::Vec2f> get_corner_tex_coordinates() override;
        parser::Vec2f get_uv(parser::Vec3f &intersected_point) override;
};


#endif
