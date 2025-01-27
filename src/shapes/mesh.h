#ifndef HW1_NEW_MESH_H
#define HW1_NEW_MESH_H

#include <memory>
#include "../parser/parser.h"
#include "shape.h"
#include "triangle.h"
#include "Eigen/Dense"


class mesh : public shape{

private:
    std::vector<parser::Vec3i> faces;
    std::vector<std::vector<parser::Vec2f>> face_tex_coords;
    std::vector<std::vector<parser::Vec3f>> face_vertices;
    Eigen::Matrix4f transformation_matrix;
    std::vector<std::shared_ptr<shape>> mesh_triangles_shape;
    std::vector<float> cdf;

public:
    int id;
    std::vector<std::shared_ptr<triangle>> mesh_triangles;
    mesh(int id, int material_id, std::vector<int> &texture_ids,
         const std::vector<parser::Material> &materials,
         const std::vector<parser::Vec3i> &faces,
         const std::vector<parser::Vec2f> &tex_coord_data,
         const std::vector<parser::Vec3f> &vertex_data,
         const std::vector<parser::Transformation> &transformations,
         const std::vector<parser::Translation> &translations,
         const std::vector<parser::Rotation> &rotations,
         const std::vector<parser::Scaling> &scalings,
         const std::vector<parser::Composite> &composites,
         int vertex_offset,
         int texture_offset,
         parser::Vec3f &radiance,
         bool is_light,
         bool is_cloud);

    float total_mesh_area;
    std::vector<parser::Vec3i>& get_faces();
    std::vector<std::vector<parser::Vec3f>>& get_face_vertices();
    std::vector<std::shared_ptr<triangle>>& get_triangles();
    std::vector<std::shared_ptr<shape>>& get_triangles_shape();
    Eigen::Matrix4f get_transformation_matrix() override;

    parser::Vec3f get_mesh_object_light_ray(float random_cdf, float random_tri1, float random_tri2);

};


#endif
