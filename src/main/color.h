#ifndef HW1_NEW_COLOR_H
#define HW1_NEW_COLOR_H
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <atomic>
#include <thread>
#include <memory>
#include "../math/math.h"
#include "../parser/parser.h"
#include "../camera/camera.h"
#include "../lights/point_light.h"
#include "../lights/area_light.h"
#include "../lights/spot_light.h"
#include "../lights/environment_light.h"
#include "../shading/BRDF.h"
#include "../shapes/mesh.h"
#include "../shapes/triangle.h"
#include "../shapes/sphere.h"
#include "../shapes/shape.h"
#include "BVH.h"
#include <random>
#include "../textures/texture.h"


parser::Vec3f compute_color(std::vector<texture> &textures,std::mt19937 &random_generator,std::vector<std::pair<float,float>> &area_samples,
                            parser::Vec3f camera_position, parser::Vec3f &ray_direction,float time,float near_distance,bool is_camera, bool ignore_np,
                            parser::Vec3f &gaze,
                            std::vector<point_light> &point_lights, std::vector<area_light> &area_lights,
                            std::vector<directional_light> &directional_lights, std::vector<spot_light> &spot_lights,
                            std::vector<environment_light> &environment_lights, std::vector<std::shared_ptr<shape>> &shapes,
                            parser::Vec3f background_color, int current_recursion_depth,int &max_recursion_depth,
                            parser::Vec3f ambient_light, float shadow_ray_epsilon, std::vector<parser::Vec3f> &triangle_normals,
                            bvh_node* bvh, int bvh_start_index,std::vector<bvh_node*> &blas_hiers, parser::Vec3i &tlas_counts,
                            std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas, std::vector<std::shared_ptr<sphere>> &spheres,
                            std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes,
                            std::vector<BRDF> &BRDFs, std::vector<float> &cloud_parameters);



parser::Vec3f ray_march(std::vector<texture> &textures,std::mt19937 &random_generator,std::vector<std::pair<float,float>> &area_samples,
                        parser::Vec3f start_position,
                        parser::Vec3f &ray_direction, float time, float near_distance,bool is_camera, bool ignore_np,parser::Vec3f &gaze,
                        std::vector<point_light> &point_lights, std::vector<area_light> &area_lights,
                        std::vector<directional_light> &directional_lights, std::vector<spot_light> &spot_lights,
                        std::vector<environment_light> &environment_lights,std::vector<std::shared_ptr<shape>> &shapes,
                        parser::Vec3f background_color, int current_recursion_depth,int &max_recursion_depth,
                        parser::Vec3f ambient_light, float shadow_ray_epsilon, std::vector<parser::Vec3f> &triangle_normals,
                        bvh_node* bvh, int bvh_start_index,std::vector<bvh_node*> &blas_hiers, parser::Vec3i &tlas_counts,
                        std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas, std::vector<std::shared_ptr<sphere>> &spheres,
                        std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes,
                        std::vector<BRDF> &BRDFs, float step_size, std::vector<float> &cloud_parameters);

#endif
