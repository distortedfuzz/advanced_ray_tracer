#ifndef ADVANCED_RAY_TRACER_PATH_TRACING_H
#define ADVANCED_RAY_TRACER_PATH_TRACING_H

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
#include "../lights/directional_light.h"
#include "../lights/environment_light.h"
#include "../shading/BRDF.h"
#include "../shapes/mesh.h"
#include "../shapes/triangle.h"
#include "../shapes/sphere.h"
#include "../shapes/shape.h"
#include "BVH.h"
#include <random>
#include "../textures/texture.h"

inline parser::Vec3f get_uniform_sample(parser::Vec3f &normal, float rand1, float rand2){

    float phi = 2 * M_PI * rand1;

    //multipliers
    float u_mult = sqrt(1 - pow(rand2, 2)) * cos(phi);
    float v_mult = sqrt(1 - pow(rand2, 2)) * sin(phi);
    float n_mult = rand2;


    //form orthonormal basis
    parser::Vec3f orth_base_min = normal;

    if(fabs(normal.x) <= fabs(normal.y) && fabs(normal.x) <= fabs(normal.z)){
        orth_base_min.x = 1.0;
    }else if(fabs(normal.y) <= fabs(normal.z) && fabs(normal.y) <= fabs(normal.x)){
        orth_base_min.y = 1.0;
    }else if(fabs(normal.z) <= fabs(normal.y) && fabs(normal.z) <= fabs(normal.x)){
        orth_base_min.z = 1.0;
    }

    orth_base_min = normalize_vector(orth_base_min);

    parser::Vec3f u_vec = normalize_vector(cross_product(normal, orth_base_min));
    parser::Vec3f v_vec = normalize_vector(cross_product(normal, u_vec));

    //multiply and return
    parser::Vec3f u_element = vector_multiply(u_vec, u_mult);
    parser::Vec3f v_element = vector_multiply(v_vec, v_mult);
    parser::Vec3f n_element = vector_multiply(normal, n_mult);

    parser::Vec3f sample_vector = vector_add(vector_add(
                            u_element, v_element), n_element);

    return normalize_vector(sample_vector);

    //multiply later by 2*pi
}


inline parser::Vec3f get_cos_importance_sample(parser::Vec3f &normal, float rand1, float rand2){

    float phi = 2 * M_PI * rand1;

    //multipliers
    float u_mult = sqrt(rand2) * cos(phi);
    float v_mult = sqrt(rand2) * sin(phi);
    float n_mult = sqrt(1 - rand2);


    //form orthonormal basis
    parser::Vec3f orth_base_min = normal;


    if(fabs(normal.x) <= fabs(normal.y) && fabs(normal.x) <= fabs(normal.z)){
        orth_base_min.x = 1.0;
    }else if(fabs(normal.y) <= fabs(normal.z) && fabs(normal.y) <= fabs(normal.x)){
        orth_base_min.y = 1.0;
    }else if(fabs(normal.z) <= fabs(normal.y) && fabs(normal.z) <= fabs(normal.x)){
        orth_base_min.z = 1.0;
    }

    orth_base_min = normalize_vector(orth_base_min);

    parser::Vec3f u_vec = normalize_vector(cross_product(normal, orth_base_min));
    parser::Vec3f v_vec = normalize_vector(cross_product(normal, u_vec));

    //multiply and return
    parser::Vec3f u_element = vector_multiply(u_vec, u_mult);
    parser::Vec3f v_element = vector_multiply(v_vec, v_mult);
    parser::Vec3f n_element = vector_multiply(normal, n_mult);

    parser::Vec3f sample_vector = vector_add(vector_add(
            u_element, v_element), n_element);

    return normalize_vector(sample_vector);
    //multiply later by pi / cos(theta), theta = asin(sqrt(rand2));
}



parser::Vec3f path_trace(std::vector<texture> &textures,std::mt19937 &random_generator,std::vector<std::pair<float,float>> &area_samples,
                            parser::Vec3f camera_position, parser::Vec3f &ray_direction,float time,float near_distance,bool is_camera, bool ignore_np,
                            parser::Vec3f &gaze,
                            std::vector<point_light> &point_lights, std::vector<area_light> &area_lights,
                            std::vector<directional_light> &directional_lights, std::vector<spot_light> &spot_lights,
                            std::vector<environment_light> &environment_lights, std::vector<std::shared_ptr<shape>> &shapes,
                            parser::Vec3f background_color, int current_recursion_depth,int max_recursion_depth,
                            int min_recursion_depth,
                            float shadow_ray_epsilon, std::vector<parser::Vec3f> &triangle_normals,
                            bvh_node* bvh, int bvh_start_index,std::vector<bvh_node*> &blas_hiers, parser::Vec3i &tlas_counts,
                            std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas, std::vector<std::shared_ptr<sphere>> &spheres,
                            std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes,
                            std::vector<BRDF> &BRDFs, camera &cam, float &throughput, int &path_depth);

#endif
