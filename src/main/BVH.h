#ifndef ADVANCED_RAY_TRACER_BVH_H
#define ADVANCED_RAY_TRACER_BVH_H

#include "../shapes/bbox.h"
#include "../shapes/shape.h"
#include "../shapes/mesh.h"
#include "../shapes/sphere.h"
#include <memory>
#include <vector>

struct hit_record {
    int primitive_id;
    float t_variable = INFINITY;
    parser::Vec3f color{-1,-1,-1};
    parser::Vec3f intersected_point;
    parser::Vec3f normal;
    std::shared_ptr<shape> intersected_shape;
    parser::Material material;
    bool is_light;
    bool is_cloud;
    parser::Vec3f radiance;
    std::vector<int> tex_ids;
    parser::Vec3f kd;
    parser::Vec3f ks;
};

struct bvh_node {
    bbox bounding_box;
    bvh_node* left;
    bvh_node* right;
    bool is_leaf = false;
    int primitive_id = -1;
    int blas_id = 0;

    bvh_node(bbox box, bvh_node* left_child, bvh_node* right_child);
    bvh_node(bbox box, int primitive);
    bvh_node(bbox box, int primitive, int blas_id);

    ~bvh_node() {
        delete left;
        delete right;
    }
};

struct bvh_node_linear {

    bbox bounding_box;
    int left_index = -1;
    int right_index = -1;
    bool is_leaf = false;
    int primitive_id = -1;

    bvh_node_linear(bbox box, int left_index, int right_index);
    bvh_node_linear(bbox box, int primitive);

};


int partition(std::vector<std::shared_ptr<shape>> &shapes, int start, int end, int partition_axis, float cut_pos, int &swap_count);
bvh_node* build_bvh(std::vector<std::shared_ptr<shape>> &shapes, int start, int end, int &count);
bool bvh_intersect(bvh_node* node, parser::Vec3f &camera_position, parser::Vec3f &ray_direction,
                   hit_record &rec, std::vector<std::shared_ptr<shape>> &shapes);

bool bvh_intersect_new(bvh_node* node, parser::Vec3f &camera_position, parser::Vec3f &ray_direction,
                       hit_record &rec, std::vector<std::shared_ptr<shape>> &shapes,std::vector<bvh_node*> &blas_hiers,
                       parser::Vec3i &tlas_counts,
                       std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas, Eigen::Matrix4f &last_transformation_matrix,std::vector<std::shared_ptr<sphere>> &spheres,
                       std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes, parser::Material &current_material);

bool bvh_intersect_mb(float time,float near_distance,bool is_camera, bool ignore_np,
                      parser::Vec3f &gaze, bvh_node* node, parser::Vec3f &camera_position, parser::Vec3f &ray_direction,
                       hit_record &rec, std::vector<std::shared_ptr<shape>> &shapes,std::vector<bvh_node*> &blas_hiers,
                       parser::Vec3i &tlas_counts,
                       std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas, Eigen::Matrix4f last_transformation_matrix,std::vector<std::shared_ptr<sphere>> &spheres,
                       std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes, parser::Material &current_material,
                       bool is_light, bool is_cloud,
                       parser::Vec3f &current_radiance,std::vector<texture> &texture,std::vector<int> &current_tex_ids);

int build_bvh_linearized(std::vector<std::shared_ptr<shape>> &shapes,
                         std::vector<bvh_node_linear> &nodes_linearized,
                         int start, int end);

bool bvh_intersect_linear(std::vector<bvh_node_linear> &nodes, int index, parser::Vec3f &camera_position,
                          parser::Vec3f &ray_direction, hit_record &rec, std::vector<std::shared_ptr<shape>> &shapes);


bvh_node* build_bvh_tlas(std::vector<std::shared_ptr<shape>> &shapes, int start, int end, int &count);
bvh_node* build_bvh_blas(std::vector<std::shared_ptr<shape>> &shapes, int start, int end, int &count);



#endif // ADVANCED_RAY_TRACER_BVH_H
