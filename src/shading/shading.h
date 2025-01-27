#ifndef HW1_NEW_SHADING_H
#define HW1_NEW_SHADING_H
#include "../math/math.h"
#include "../parser/parser.h"
#include "../shapes/shape.h"
#include "../shapes/triangle.h"
#include "../shapes/mesh.h"
#include "../shapes/sphere.h"
#include <cmath>
#include <vector>
#include <random>
#include "../main/BVH.h"
#include <iostream>

inline parser::Vec3f ambient_light(const parser::Vec3f &ambient_reflectance,
                                   const parser::Vec3f &ambient_light){

    parser::Vec3f result;
    result.x = ambient_reflectance.x* ambient_light.x;
    result.y = ambient_reflectance.y* ambient_light.y;
    result.z = ambient_reflectance.z* ambient_light.z;
    return result;
}

inline parser::Vec3f diffuse_light_radiance_env(const parser::Vec3f &normal,
                                            const parser::Vec3f diffuse_reflectance,
                                            const parser::Vec3f light_direction,
                                            const parser::Vec3f radiance){


    float cost = dot_product(normal, light_direction);
    parser::Vec3f result;

    cost = fmax(cost, 0.0f);

    result.x = diffuse_reflectance.x * cost * radiance.x;
    result.y = diffuse_reflectance.y * cost * radiance.y;
    result.z = diffuse_reflectance.z * cost * radiance.z;


    return result;
}


inline parser::Vec3f specular_light_radiance_env(const parser::Vec3f &normal,
                                             const parser::Vec3f specular_reflectance,
                                             float phong_exponent,
                                             const parser::Vec3f light_direction,
                                             const parser::Vec3f light_radiance,
                                             const parser::Vec3f &point,
                                             const parser::Vec3f &cam_position){



    parser::Vec3f cam_vector = vector_divide(vector_subtract(cam_position, point), get_vector_magnitude(vector_subtract(cam_position, point)));

    parser::Vec3f half_vector = vector_divide(vector_add(light_direction, cam_vector), get_vector_magnitude(
            vector_add(light_direction, cam_vector)));


    float cosa = dot_product(normal, half_vector)/ (get_vector_magnitude(normal)* get_vector_magnitude(half_vector));

    parser::Vec3f result;

    if(acos(dot_product(normal, light_direction))>3.14/2){

        result.x = 0.0;
        result.y = 0.0;
        result.z = 0.0;
        return result;
    }

    result.x = specular_reflectance.x * pow(cosa, phong_exponent) * light_radiance.x;
    result.y = specular_reflectance.y * pow(cosa, phong_exponent) * light_radiance.y;
    result.z = specular_reflectance.z * pow(cosa, phong_exponent) * light_radiance.z;

    return result;
}

inline parser::Vec3f diffuse_light_radiance(const parser::Vec3f &normal,
                                            const parser::Vec3f diffuse_reflectance,
                                            const parser::Vec3f light_direction,
                                            const parser::Vec3f radiance){


    float cost = dot_product(normal, vector_multiply(light_direction,-1));
    parser::Vec3f result;

    cost = fmax(cost, 0.0f);
    result.x = diffuse_reflectance.x * cost * radiance.x;
    result.y = diffuse_reflectance.y * cost * radiance.y;
    result.z = diffuse_reflectance.z * cost * radiance.z;


    return result;
}


inline parser::Vec3f specular_light_radiance(const parser::Vec3f &normal,
                                    const parser::Vec3f specular_reflectance,
                                    float phong_exponent,
                                    const parser::Vec3f light_direction,
                                    const parser::Vec3f light_radiance,
                                    const parser::Vec3f &point,
                                    const parser::Vec3f &cam_position){



    parser::Vec3f cam_vector = vector_divide(vector_subtract(cam_position, point), get_vector_magnitude(vector_subtract(cam_position, point)));

    parser::Vec3f half_vector = vector_divide(vector_add(vector_multiply(light_direction,-1), cam_vector), get_vector_magnitude(
            vector_add(vector_multiply(light_direction, -1), cam_vector)));


    float cosa = dot_product(normal, half_vector)/ (get_vector_magnitude(normal)* get_vector_magnitude(half_vector));

    parser::Vec3f result;

    if(acos(dot_product(normal, light_direction))>3.14/2){

        result.x = 0.0;
        result.y = 0.0;
        result.z = 0.0;
        return result;
    }

    result.x = specular_reflectance.x * pow(cosa, phong_exponent) * light_radiance.x;
    result.y = specular_reflectance.y * pow(cosa, phong_exponent) * light_radiance.y;
    result.z = specular_reflectance.z * pow(cosa, phong_exponent) * light_radiance.z;

    return result;
}


inline parser::Vec3f diffuse_light(const parser::Vec3f &normal,
                                   const parser::Vec3f diffuse_reflectance,
                                   const parser::Vec3f light_position,
                                   const parser::Vec3f light_intensity,
                                   const parser::Vec3f &point){


    parser::Vec3f point_light_direction = vector_divide(vector_subtract(light_position, point), get_vector_magnitude(vector_subtract(light_position, point)));

    float distance = get_vector_magnitude(vector_subtract(light_position, point));

    float cost = dot_product(normal, point_light_direction)/ (get_vector_magnitude(normal)* get_vector_magnitude(point_light_direction));
    parser::Vec3f result;

    cost = fmax(cost, 0.0f);
    result.x = diffuse_reflectance.x * cost * (light_intensity.x / pow(distance,2));
    result.y = diffuse_reflectance.y * cost * (light_intensity.y / pow(distance,2));
    result.z = diffuse_reflectance.z * cost * (light_intensity.z / pow(distance,2));


    return result;
}


inline parser::Vec3f specular_light(const parser::Vec3f &normal,
                                    const parser::Vec3f specular_reflectance,
                                    float phong_exponent,
                                    const parser::Vec3f light_position,
                                    const parser::Vec3f light_intensity,
                                    const parser::Vec3f &point,
                                    const parser::Vec3f &cam_position){



    parser::Vec3f cam_vector = vector_divide(vector_subtract(cam_position, point), get_vector_magnitude(vector_subtract(cam_position, point)));


    parser::Vec3f point_light_direction = vector_divide(vector_subtract(light_position, point), get_vector_magnitude(vector_subtract(light_position, point)));

    parser::Vec3f half_vector = vector_divide(vector_add(point_light_direction, cam_vector), get_vector_magnitude(
            vector_add(point_light_direction, cam_vector)));

    float distance = get_vector_magnitude(vector_subtract(light_position, point));

    float cosa = dot_product(normal, half_vector)/ (get_vector_magnitude(normal)* get_vector_magnitude(half_vector));

    parser::Vec3f result;

    if(acos(dot_product(normal, point_light_direction)/ (get_vector_magnitude(normal)* get_vector_magnitude(point_light_direction)))>3.14/2){

        result.x = 0.0;
        result.y = 0.0;
        result.z = 0.0;
        return result;
    }

    result.x = specular_reflectance.x * pow(cosa, phong_exponent) * (light_intensity.x / pow(distance,2));
    result.y = specular_reflectance.y * pow(cosa, phong_exponent) * (light_intensity.y / pow(distance,2));
    result.z = specular_reflectance.z * pow(cosa, phong_exponent) * (light_intensity.z / pow(distance,2));

    return result;
}


inline parser::Vec3f get_brdf_result(const parser::Vec3f &light_position,
                                     const parser::Vec3f &intersected_point,
                                     const parser::Vec3f &light_intensity,
                                     const parser::Vec3f &BRDF){

    parser::Vec3f result;

    float distance_sq = pow(get_vector_magnitude(vector_subtract(light_position, intersected_point)), 2.0f);
    distance_sq = std::max(distance_sq, 1e-6f);

    result.x = BRDF.x * (light_intensity.x / distance_sq);
    result.y = BRDF.y * (light_intensity.y / distance_sq);
    result.z = BRDF.z * (light_intensity.z / distance_sq);

    return result;
}

inline parser::Vec3f get_brdf_result_new(const parser::Vec3f &radiance,
                                         const parser::Vec3f &BRDF){

    parser::Vec3f result;

    result.x = BRDF.x * radiance.x;
    result.y = BRDF.y * radiance.y;
    result.z = BRDF.z * radiance.z;

    return result;
}


inline bool point_in_shadow(float time,float near_distance,bool is_camera,
                            bool ignore_np,
                            parser::Vec3f &gaze,parser::Vec3f &point,
                            const parser::Vec3f &normal,
                            float shadow_ray_epsilon,
                            const parser::Vec3f light_position,
                            std::vector<std::shared_ptr<shape>> &shapes,
                            bvh_node* bvh,
                            int bvh_start_index,
                            std::vector<bvh_node*> &blas_hiers, parser::Vec3i &tlas_counts,
                            std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas,std::vector<std::shared_ptr<sphere>> &spheres,
                            std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes){


    parser::Vec3f offset_point = vector_add(point, vector_multiply(normal, shadow_ray_epsilon + 0.0005));

    float light_distance = get_vector_magnitude(vector_subtract(light_position, offset_point));
    parser::Vec3f point_light_vector = vector_divide(vector_subtract(light_position, offset_point), light_distance);


    hit_record new_rec{-1, 0.0};
    Eigen::Matrix4f holder_matrix = Eigen::Matrix4f::Identity();
    parser::Material hold_mat;
    std::vector<int> hold_tex;
    bool is_light_holder = false;
    bool is_cloud_holder = false;
    parser::Vec3f hold_radiance;
    std::vector<texture> tex_hold;

    bool intersection = bvh_intersect_mb(time, near_distance, is_camera, ignore_np,
                                         gaze,bvh, offset_point,point_light_vector ,new_rec,
                                          shapes, blas_hiers,
                                          tlas_counts,primitives_inside_blas,
                                          holder_matrix,spheres, triangles, meshes,hold_mat,
                                          is_light_holder, is_cloud_holder,
                                          hold_radiance,tex_hold,hold_tex);

    if( intersection && new_rec.t_variable* get_vector_magnitude(point_light_vector) < light_distance){
        return true;
    }else{
        return false;
    }

}


inline bool point_in_shadow_in_check(float time,float near_distance,bool is_camera,
                            bool ignore_np,
                            parser::Vec3f &gaze,parser::Vec3f &point,
                            const parser::Vec3f &normal,
                            float shadow_ray_epsilon,
                            const parser::Vec3f light_position,
                            std::vector<std::shared_ptr<shape>> &shapes,
                            bvh_node* bvh,
                            int bvh_start_index,
                            std::vector<bvh_node*> &blas_hiers, parser::Vec3i &tlas_counts,
                            std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas,std::vector<std::shared_ptr<sphere>> &spheres,
                            std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes){


    parser::Vec3f offset_point = vector_add(point, vector_multiply(normal, shadow_ray_epsilon + 0.0005));

    float light_distance = get_vector_magnitude(vector_subtract(light_position, offset_point));
    parser::Vec3f point_light_vector = vector_divide(vector_subtract(light_position, offset_point), light_distance);


    hit_record new_rec{-1, 0.0};
    new_rec.is_light = false;
    Eigen::Matrix4f holder_matrix = Eigen::Matrix4f::Identity();
    parser::Material hold_mat;
    std::vector<int> hold_tex;
    bool is_light_holder = false;
    bool is_cloud_holder = false;
    parser::Vec3f hold_radiance;
    std::vector<texture> tex_hold;

    bool intersection = bvh_intersect_mb(time, near_distance, is_camera, ignore_np,
                                         gaze,bvh, offset_point,point_light_vector ,new_rec,
                                         shapes, blas_hiers,
                                         tlas_counts,primitives_inside_blas,
                                         holder_matrix,spheres, triangles, meshes,hold_mat,
                                         is_light_holder, is_cloud_holder,
                                         hold_radiance,tex_hold,hold_tex);

    if( intersection && new_rec.t_variable* get_vector_magnitude(point_light_vector) < light_distance && !new_rec.is_light){
        return true;
    }else{
        return false;
    }

}


inline bool directional_light_point_in_shadow(float time,float near_distance,bool is_camera, bool ignore_np,
                            parser::Vec3f &gaze,parser::Vec3f &point,
                            const parser::Vec3f &normal,
                            float shadow_ray_epsilon,
                            const parser::Vec3f light_direction,
                            std::vector<std::shared_ptr<shape>> &shapes,
                            bvh_node* bvh,
                            int bvh_start_index,
                            std::vector<bvh_node*> &blas_hiers, parser::Vec3i &tlas_counts,
                            std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas,std::vector<std::shared_ptr<sphere>> &spheres,
                            std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes){


    parser::Vec3f offset_point = vector_add(point, vector_multiply(normal, shadow_ray_epsilon + 1e-5));

    parser::Vec3f point_light_vector = vector_multiply(light_direction, -1);


    hit_record new_rec{-1, 0.0};
    Eigen::Matrix4f holder_matrix = Eigen::Matrix4f::Identity();
    parser::Material hold_mat;
    std::vector<int> hold_tex;
    bool is_light_holder = false;
    bool is_cloud_holder = false;
    parser::Vec3f hold_radiance;
    std::vector<texture> tex_hold;

    bool intersection = bvh_intersect_mb(time, near_distance, is_camera,ignore_np ,gaze,bvh, offset_point,point_light_vector ,new_rec,
                                         shapes, blas_hiers,
                                         tlas_counts,primitives_inside_blas, holder_matrix,spheres,
                                         triangles, meshes,hold_mat,
                                         is_light_holder, is_cloud_holder,
                                         hold_radiance,tex_hold,hold_tex);

    if(intersection && new_rec.t_variable > 0.f){
        return true;
    }else{
        return false;
    }

}




inline parser::Vec3f get_glossy_reflection_ray(std::mt19937 &gRandomGenerator,parser::Vec3f reflection_ray, float roughness){

    parser::Vec3f hold_reflection_ray{0,0,0};

    int min_abs_id = 0;
    if(fabs(reflection_ray.x) < fabs(reflection_ray.y) && fabs(reflection_ray.x) < fabs(reflection_ray.z)){
        min_abs_id = 1;
    }else if(fabs(reflection_ray.z) < fabs(reflection_ray.y) && fabs(reflection_ray.z) < fabs(reflection_ray.x)){
        min_abs_id = 3;
    }else{
        min_abs_id = 2;
    }



    if(min_abs_id == 1){
        hold_reflection_ray.x = 1.0;
    }else if(min_abs_id == 2){
        hold_reflection_ray.y = 1.0;
    }else if(min_abs_id == 3){
        hold_reflection_ray.z = 1.0;
    }


    hold_reflection_ray = normalize_vector(hold_reflection_ray);

    parser::Vec3f u_vec = normalize_vector(cross_product(reflection_ray, hold_reflection_ray));
    parser::Vec3f v_vec = normalize_vector(cross_product(reflection_ray, u_vec));

    parser::Vec3f real_ray{0,0,0};


    std::uniform_real_distribution<> gNURandomDistribution(-0.5f, 0.5f);
    float rand1 = gNURandomDistribution(gRandomGenerator);
    float rand2 = gNURandomDistribution(gRandomGenerator);
    //std::cout<<rand1<<" "<<rand2<<std::endl;
    real_ray = normalize_vector(vector_add(reflection_ray,
                                           vector_multiply(
                                                   vector_add(
                                                           vector_multiply(u_vec, rand1),
                                                           vector_multiply(v_vec, rand2)),
                                                   roughness)));

    return real_ray;

}
#endif
