#include <vector>
#include <memory>
#include "../math/math.h"
#include "../lights/point_light.h"
#include "../lights/area_light.h"
#include "../lights/directional_light.h"
#include "../lights/spot_light.h"
#include "../lights/environment_light.h"
#include "../shading/BRDF.h"
#include "../shapes/mesh.h"
#include "../shading/shading.h"
#include "../shading/refraction.h"
#include "BVH.h"
#include <iostream>


parser::Vec3f get_ambient_light(parser::Vec3f ambient_coeff, parser::Vec3f &ambient_light){

    parser::Vec3f result{ambient_coeff.x* ambient_light.x,
                         ambient_coeff.y* ambient_light.y,
                         ambient_coeff.z* ambient_light.z};

    return result;

}

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
                        std::vector<BRDF> &BRDFs, float step_size, std::vector<float> &cloud_parameters){

    hit_record new_rec{-1, 0.0};
    Eigen::Matrix4f starting_matrix = Eigen::Matrix4f::Identity();
    parser::Material hold_mat;
    bool is_light_holder = false;
    bool is_cloud_holder = false;
    parser::Vec3f hold_radiance;
    std::vector<int> hold_tex;
    parser::Vec3f hold{0,0,0};

    parser::Vec3f eps_int = vector_add(start_position, vector_multiply(ray_direction, 1e-4));


    bool intersection = bvh_intersect_mb(0,0, false, false,
                                         hold,bvh, eps_int,
                                         ray_direction ,new_rec,
                                         shapes, blas_hiers,
                                         tlas_counts,primitives_inside_blas, starting_matrix,
                                         spheres, triangles, meshes, hold_mat,
                                         is_light_holder, is_cloud_holder,hold_radiance,
                                         textures,hold_tex);


    float absorption_constant = cloud_parameters[0];
    float scattering_constant = cloud_parameters[1];
    float asymmetry_parameter = cloud_parameters[2];

    float extinction_coefficient = 0.0;
    float scattering_coefficient = 0.0;
    float absorption_coefficient = 0.0;

    if(intersection){
        if(new_rec.tex_ids.size() < 1){
            return parser::Vec3f{0,0,0};
        }

        float inside_length = get_vector_magnitude(vector_subtract(start_position, new_rec.intersected_point));

        float distance_taken = 0.0f;
        parser::Vec3f total_march_value{0,0,0};
        float transmittance = 1.0f;

        while(distance_taken < inside_length){
            distance_taken += step_size;
            parser::Vec3f this_march_value{0,0,0};
            parser::Vec3f current_point = vector_add(start_position, vector_multiply(
                    ray_direction, distance_taken));

            this_march_value = textures[new_rec.tex_ids[0]].get_texture_value_perlin(current_point);

            if(this_march_value.x < 0.3){
                continue;
            }

            absorption_coefficient = absorption_constant * this_march_value.x;
            scattering_coefficient = scattering_constant * this_march_value.x;
            extinction_coefficient = absorption_coefficient + scattering_coefficient;

            float light_final = 0.0f;
            for(auto &light: point_lights){
                parser::Vec3f light_dir = normalize_vector(vector_subtract(light.get_position(), current_point));

                hit_record light_new_rec{-1, 0.0};
                Eigen::Matrix4f light_starting_matrix = Eigen::Matrix4f::Identity();
                parser::Material light_hold_mat;
                bool is_light_holder_light = false;
                bool is_cloud_holder_light = false;
                parser::Vec3f hold_radiance_light;
                std::vector<int> hold_tex_light;
                parser::Vec3f hold_light{0,0,0};

                parser::Vec3f eps_int_light = vector_add(current_point, vector_multiply(light_dir, 1e-4));


                bool light_intersection = bvh_intersect_mb(0,0, false, false,
                                                     hold_light,bvh, eps_int_light,
                                                     light_dir ,light_new_rec,
                                                     shapes, blas_hiers,
                                                     tlas_counts,primitives_inside_blas, light_starting_matrix,
                                                     spheres, triangles, meshes, light_hold_mat,
                                                     is_light_holder_light, is_cloud_holder_light,hold_radiance_light,
                                                     textures,hold_tex_light);

                float light_inside_length = get_vector_magnitude(vector_subtract(current_point, light_new_rec.intersected_point));

                float light_distance_taken = 0.0f;
                float density_sum = 0.0;

                while(light_distance_taken < light_inside_length){
                    light_distance_taken += step_size;
                    parser::Vec3f light_march_value{0,0,0};
                    parser::Vec3f light_current_point = vector_add(current_point, vector_multiply(
                            light_dir, light_distance_taken));

                    light_march_value = textures[new_rec.tex_ids[0]].get_texture_value_perlin(light_current_point);

                    density_sum += light_march_value.x;

                }

                float light_transmission = exp(-step_size * density_sum * extinction_coefficient);

                float phase = phase_function(asymmetry_parameter, dot_product(vector_multiply(ray_direction,1)
                                                              , light_dir));

                light_final = light_final + phase * light_transmission * light.get_intensity().x /
                        pow(get_vector_magnitude(vector_subtract(light.get_position(),current_point)),scattering_coefficient / extinction_coefficient);

            }

            transmittance *= exp(-step_size * this_march_value.x * extinction_coefficient);
            //std::cout<<this_march_value.x*transmittance<<std::endl;

            total_march_value  = vector_add(total_march_value,
                                            vector_multiply(this_march_value , light_final*
                                                                               transmittance));



        }

        parser::Vec3f out_point = vector_add(start_position, vector_multiply(ray_direction, inside_length + 1e-4));

        parser::Vec3f color_value = compute_color(textures,random_generator,area_samples,out_point,
                                                  ray_direction, time,
                                                  near_distance,false,ignore_np,gaze,
                                                  point_lights, area_lights, directional_lights, spot_lights,
                                                  environment_lights,shapes,
                                                  background_color, current_recursion_depth,
                                                  max_recursion_depth, ambient_light,
                                                  shadow_ray_epsilon, triangle_normals, bvh,
                                                  bvh_start_index, blas_hiers,
                                                  tlas_counts,primitives_inside_blas, spheres, triangles, meshes, BRDFs,
                                                  cloud_parameters);

        //std::cout<<transmittance<<std::endl;
        //std::cout<<color_value.x<<" "<<color_value.y<<" "<<color_value.z<<std::endl;
        return vector_add(vector_multiply(color_value, transmittance), total_march_value);
    }else{
        parser::Vec3f color_value = compute_color(textures,random_generator,area_samples,start_position,
                                                  ray_direction, time,
                                                  near_distance,false,ignore_np,gaze,
                                                  point_lights, area_lights, directional_lights, spot_lights,
                                                  environment_lights,shapes,
                                                  background_color, current_recursion_depth,
                                                  max_recursion_depth, ambient_light,
                                                  shadow_ray_epsilon, triangle_normals, bvh,
                                                  bvh_start_index, blas_hiers,
                                                  tlas_counts,primitives_inside_blas, spheres, triangles, meshes, BRDFs,
                                                  cloud_parameters);

        return color_value;
    }



}




parser::Vec3f compute_color(std::vector<texture> &textures,std::mt19937 &random_generator,std::vector<std::pair<float,float>> &area_samples, parser::Vec3f camera_position,
                            parser::Vec3f &ray_direction, float time, float near_distance,bool is_camera, bool ignore_np,parser::Vec3f &gaze,
                            std::vector<point_light> &point_lights, std::vector<area_light> &area_lights,
                            std::vector<directional_light> &directional_lights, std::vector<spot_light> &spot_lights,
                            std::vector<environment_light> &environment_lights,std::vector<std::shared_ptr<shape>> &shapes,
                            parser::Vec3f background_color, int current_recursion_depth,int &max_recursion_depth,
                            parser::Vec3f ambient_light, float shadow_ray_epsilon, std::vector<parser::Vec3f> &triangle_normals,
                            bvh_node* bvh, int bvh_start_index,std::vector<bvh_node*> &blas_hiers, parser::Vec3i &tlas_counts,
                            std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas, std::vector<std::shared_ptr<sphere>> &spheres,
                            std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes,
                            std::vector<BRDF> &BRDFs, std::vector<float> &cloud_parameters){

    parser::Vec3f result{0,0,0};

    if(current_recursion_depth == max_recursion_depth+1){
        return result;
    }

    current_recursion_depth++;


    hit_record new_rec{-1, 0.0};
    Eigen::Matrix4f starting_matrix = Eigen::Matrix4f::Identity();
    parser::Material hold_mat;
    bool is_light_holder = false;
    bool is_cloud_holder = false;
    parser::Vec3f hold_radiance;
    std::vector<int> hold_tex;

    bool intersection = bvh_intersect_mb(time,near_distance, is_camera, ignore_np, gaze,bvh, camera_position,ray_direction ,new_rec,
                                          shapes, blas_hiers,
                                          tlas_counts,primitives_inside_blas, starting_matrix, spheres, triangles, meshes, hold_mat,
                                          is_light_holder, is_cloud_holder, hold_radiance,textures,hold_tex);


    parser::Vec3f intersected_point = new_rec.intersected_point;
    auto intersected_shape = new_rec.intersected_shape;
    parser::Vec3f normal = new_rec.normal;

    parser::Vec3f kd = new_rec.kd;
    parser::Vec3f ks = new_rec.ks;


    bool total_internal_reflection = false;

    if(intersection){
        if(is_camera && new_rec.is_light){
            return new_rec.radiance;
        }

        if(new_rec.is_cloud){

            return ray_march(textures,random_generator,area_samples,new_rec.intersected_point,
                             ray_direction, time,
                             near_distance,false,ignore_np,gaze,
                             point_lights, area_lights, directional_lights, spot_lights,
                             environment_lights,shapes,
                             background_color, current_recursion_depth,
                             max_recursion_depth, ambient_light,
                             shadow_ray_epsilon, triangle_normals, bvh,
                             bvh_start_index, blas_hiers,
                             tlas_counts,primitives_inside_blas, spheres, triangles, meshes, BRDFs, 0.1,
                             cloud_parameters);

        }


        if(new_rec.material.is_mirror){

            parser::Vec3f reflection_ray = vector_add(vector_multiply(normal, -2 * dot_product(normal, ray_direction)),
                                                      ray_direction);
            if(new_rec.material.roughness > 1e-6){

                reflection_ray = get_glossy_reflection_ray(random_generator,reflection_ray, new_rec.material.roughness);

            }
            parser::Vec3f normalized_reflection_ray = normalize_vector(reflection_ray);
            parser::Vec3f epsilon_point = vector_add(intersected_point, vector_multiply(normal, shadow_ray_epsilon));

            parser::Vec3f mirror_color = compute_color(textures,random_generator,area_samples,epsilon_point, normalized_reflection_ray, time,
                                                       near_distance,false,ignore_np,gaze,
                                                       point_lights, area_lights, directional_lights, spot_lights,
                                                       environment_lights,shapes,
                                                       background_color, current_recursion_depth,
                                                       max_recursion_depth, ambient_light,
                                                       shadow_ray_epsilon, triangle_normals, bvh,
                                                       bvh_start_index, blas_hiers,
                                                       tlas_counts,primitives_inside_blas, spheres, triangles, meshes, BRDFs,
                                                       cloud_parameters);


            result.x += (mirror_color.x * new_rec.material.mirror.x);
            result.y += (mirror_color.y * new_rec.material.mirror.y);
            result.z += (mirror_color.z * new_rec.material.mirror.z);


        }else if(new_rec.material.is_dielectric){

            //check if ray is inside the object
            float ray_normal_dot = dot_product(ray_direction, normal);

            //calculate the reflection ray in a simple way and normalize it
            parser::Vec3f reflection_ray = vector_add(vector_multiply(normal, -2 * dot_product(normal, ray_direction)),
                                                      ray_direction);
            if(new_rec.material.roughness > 1e-6){

                reflection_ray = get_glossy_reflection_ray(random_generator,reflection_ray, new_rec.material.roughness);

            }

            parser::Vec3f normalized_reflection_ray = normalize_vector(reflection_ray);


            if(ray_normal_dot>=0.0){
                //ray inside object
                parser::Vec3f neg_normal = vector_multiply(normal,-1);

                //calculate cos_t and cos_a for fresnel calculation
                float cos_t = get_cos_t(vector_multiply(ray_direction, -1), neg_normal);

                //after the reflection calculations, check if there is total internal reflection
                float in_a = inside_cos_a(cos_t,new_rec.material.refraction_index,1.0);


                float beers_distance = get_vector_magnitude(vector_subtract(intersected_point, camera_position));

                parser::Vec3f starting_luminance{1.0, 1.0, 1.0};


                parser::Vec3f attenuation_loss = attenuation(starting_luminance,new_rec.material.absorption_coefficient, beers_distance);


                if(in_a >= 0.0){

                    float cos_a = get_cos_a(cos_t, new_rec.material.refraction_index,1.0);



                    fresnel dielectric_fresnel_ratios = get_fresnel_dielectric(cos_t, cos_a, new_rec.material.refraction_index,1.0);

                    //get refraction ray
                    parser::Vec3f dielectric_refraction_ray = refraction_ray(ray_direction,
                                                                             neg_normal,
                                                                             new_rec.material.refraction_index, 1.0);

                    parser::Vec3f normalized_dielectric_refraction_ray = normalize_vector(dielectric_refraction_ray);


                    if(new_rec.material.roughness > 1e-6){

                        normalized_dielectric_refraction_ray = get_glossy_reflection_ray(random_generator,normalized_dielectric_refraction_ray, new_rec.material.roughness);

                    }

                    parser::Vec3f refraction_in_epsilon_point = vector_add(intersected_point,
                                                                           vector_multiply(normal, shadow_ray_epsilon));




                    parser::Vec3f transmission_color = compute_color(textures,random_generator,area_samples,refraction_in_epsilon_point, normalized_dielectric_refraction_ray,
                                                                     time, near_distance,false, ignore_np,gaze,
                                                                     point_lights,area_lights, directional_lights,spot_lights,
                                                                     environment_lights,shapes, background_color,
                                                                     current_recursion_depth,
                                                                     max_recursion_depth, ambient_light,
                                                                     shadow_ray_epsilon, triangle_normals, bvh,
                                                                     bvh_start_index, blas_hiers,
                                                                     tlas_counts,primitives_inside_blas, spheres, triangles, meshes, BRDFs,
                                                                     cloud_parameters);

                    result.x += (transmission_color.x * dielectric_fresnel_ratios.transmission_ratio * attenuation_loss.x);
                    result.y += (transmission_color.y * dielectric_fresnel_ratios.transmission_ratio * attenuation_loss.y);
                    result.z += (transmission_color.z * dielectric_fresnel_ratios.transmission_ratio * attenuation_loss.z);

                    parser::Vec3f reflection_epsilon_point = vector_add(intersected_point,
                                                                        vector_multiply(neg_normal, shadow_ray_epsilon));


                    parser::Vec3f reflection_color = compute_color(textures,random_generator,area_samples,reflection_epsilon_point, normalized_reflection_ray,time, near_distance,
                                                                   false, ignore_np,gaze,
                                                                   point_lights,area_lights, directional_lights,spot_lights,
                                                                   environment_lights, shapes, background_color,
                                                                   current_recursion_depth,
                                                                   max_recursion_depth, ambient_light,
                                                                   shadow_ray_epsilon, triangle_normals, bvh,
                                                                   bvh_start_index,blas_hiers,
                                                                   tlas_counts,primitives_inside_blas, spheres, triangles, meshes, BRDFs,
                                                                   cloud_parameters);

                    result.x += (reflection_color.x * dielectric_fresnel_ratios.reflection_ratio * attenuation_loss.x);
                    result.y += (reflection_color.y * dielectric_fresnel_ratios.reflection_ratio * attenuation_loss.y);
                    result.z += (reflection_color.z * dielectric_fresnel_ratios.reflection_ratio * attenuation_loss.z);




                }else{
                    total_internal_reflection = true;
                    parser::Vec3f reflection_epsilon_point = vector_add(intersected_point,
                                                                        vector_multiply(neg_normal, shadow_ray_epsilon));


                    parser::Vec3f reflection_color = compute_color(textures,random_generator,area_samples,reflection_epsilon_point, normalized_reflection_ray,time, near_distance,
                                                                   false, ignore_np, gaze,
                                                                   point_lights,area_lights,directional_lights,spot_lights,
                                                                   environment_lights, shapes, background_color,
                                                                   current_recursion_depth,
                                                                   max_recursion_depth, ambient_light,
                                                                   shadow_ray_epsilon,triangle_normals, bvh,
                                                                   bvh_start_index,blas_hiers,
                                                                   tlas_counts,primitives_inside_blas, spheres, triangles, meshes, BRDFs,
                                                                   cloud_parameters);

                    result.x += (reflection_color.x * attenuation_loss.x);
                    result.y += (reflection_color.y * attenuation_loss.y);
                    result.z += (reflection_color.z * attenuation_loss.z);

                }


            }else{

                //ray outside object
                float cos_t = get_cos_t(vector_multiply(ray_direction, -1), normal);
                float cos_a = get_cos_a(cos_t, 1.0,new_rec.material.refraction_index);

                fresnel dielectric_fresnel_ratios = get_fresnel_dielectric(cos_t, cos_a, 1.0,new_rec.material.refraction_index);

                //prepare the intersection point with
                parser::Vec3f reflection_epsilon_point = vector_add(intersected_point,
                                                                    vector_multiply(normal, shadow_ray_epsilon));

                parser::Vec3f reflection_color = compute_color(textures,random_generator,area_samples,reflection_epsilon_point, normalized_reflection_ray,time, near_distance,
                                                               false, ignore_np, gaze,
                                                               point_lights,area_lights, directional_lights,spot_lights,
                                                               environment_lights, shapes, background_color,
                                                               current_recursion_depth,
                                                               max_recursion_depth, ambient_light,
                                                               shadow_ray_epsilon,triangle_normals, bvh,
                                                               bvh_start_index,blas_hiers,
                                                               tlas_counts,primitives_inside_blas, spheres, triangles, meshes, BRDFs,
                                                               cloud_parameters);

                result.x += (reflection_color.x * dielectric_fresnel_ratios.reflection_ratio);
                result.y += (reflection_color.y * dielectric_fresnel_ratios.reflection_ratio);
                result.z += (reflection_color.z * dielectric_fresnel_ratios.reflection_ratio);

                parser::Vec3f dielectric_refraction_ray = refraction_ray(ray_direction, normal, 1.0,new_rec.material.refraction_index);
                parser::Vec3f normalized_dielectric_refraction_ray = normalize_vector(dielectric_refraction_ray);

                if(new_rec.material.roughness > 1e-6){

                    normalized_dielectric_refraction_ray = get_glossy_reflection_ray(random_generator,normalized_dielectric_refraction_ray, new_rec.material.roughness);

                }

                parser::Vec3f refraction_in_epsilon_point = vector_add(intersected_point,
                                                                       vector_multiply(normal, -1 * shadow_ray_epsilon));


                parser::Vec3f refraction_color = compute_color(textures,random_generator,area_samples,refraction_in_epsilon_point, normalized_dielectric_refraction_ray,time,
                                                               near_distance,false, ignore_np, gaze,
                                                               point_lights, area_lights, directional_lights,spot_lights,
                                                               environment_lights, shapes, background_color, current_recursion_depth,
                                                               max_recursion_depth, ambient_light, shadow_ray_epsilon,triangle_normals, bvh,
                                                               bvh_start_index,blas_hiers,
                                                               tlas_counts,primitives_inside_blas, spheres, triangles, meshes, BRDFs,
                                                               cloud_parameters);

                result.x += (refraction_color.x * dielectric_fresnel_ratios.transmission_ratio);
                result.y += (refraction_color.y * dielectric_fresnel_ratios.transmission_ratio);
                result.z += (refraction_color.z * dielectric_fresnel_ratios.transmission_ratio);


            }


        }else if(new_rec.material.is_conductor){

            parser::Vec3f reflection_ray = vector_add(vector_multiply(normal, -2 * dot_product(normal, ray_direction)),
                                                      ray_direction);
            parser::Vec3f conductor_color;
            fresnel conductor_fresnel_ratios;
            parser::Vec3f og_ray = reflection_ray;

            if(new_rec.material.roughness > 1e-6){
                reflection_ray = get_glossy_reflection_ray(random_generator,reflection_ray, new_rec.material.roughness);
            }
            parser::Vec3f normalized_reflection_ray = normalize_vector(reflection_ray);
            parser::Vec3f epsilon_point = vector_add(intersected_point,
                                                     vector_multiply(normalized_reflection_ray, shadow_ray_epsilon));


            float cos_t = get_cos_t(og_ray, normal);

            conductor_fresnel_ratios = get_fresnel_conductor(cos_t, new_rec.material.refraction_index,
                                                             new_rec.material.absorption_index);

            conductor_color = compute_color(textures,random_generator,area_samples,epsilon_point, normalized_reflection_ray,time,
                                            near_distance,false, ignore_np, gaze,
                                            point_lights,area_lights, directional_lights,spot_lights,
                                            environment_lights, shapes, background_color,
                                            current_recursion_depth,
                                            max_recursion_depth, ambient_light,
                                            shadow_ray_epsilon, triangle_normals, bvh, bvh_start_index,blas_hiers,
                                            tlas_counts,primitives_inside_blas, spheres, triangles, meshes, BRDFs,
                                            cloud_parameters);

            result.x += (conductor_color.x * new_rec.material.mirror.x)*conductor_fresnel_ratios.reflection_ratio;
            result.y += (conductor_color.y * new_rec.material.mirror.y)*conductor_fresnel_ratios.reflection_ratio;
            result.z += (conductor_color.z * new_rec.material.mirror.z)*conductor_fresnel_ratios.reflection_ratio;

        }

        if(!total_internal_reflection){

            if(new_rec.color.x != -1 || new_rec.color.y != -1 || new_rec.color.z != -1){
                return new_rec.color;
            }

            parser::Vec3f ka = new_rec.material.ambient;
            for(auto & light: point_lights){


                bool in_shadow = point_in_shadow(time, near_distance, false, ignore_np,gaze,intersected_point, normal, shadow_ray_epsilon, light.get_position(),
                                                 shapes, bvh, bvh_start_index, blas_hiers,
                                                 tlas_counts, primitives_inside_blas,spheres, triangles, meshes);

                if(!in_shadow){

                    if(new_rec.material.BRDF_id == -1){

                        parser::Vec3f  diffuse_l = diffuse_light(normal, kd,
                                                                 light.get_position(), light.get_intensity(),
                                                                 intersected_point);

                        parser::Vec3f specular_l = specular_light(normal, ks,
                                                                  new_rec.material.phong_exponent,light.get_position(),
                                                                  light.get_intensity(), intersected_point, camera_position);


                        result.x += (diffuse_l.x + specular_l.x);
                        result.y += (diffuse_l.y + specular_l.y);
                        result.z += (diffuse_l.z + specular_l.z);

                    }else{
                        parser::Vec3f brdf_result{0,0,0};

                        //GET THE BRDF WITH THE CORRECT ID
                        for(auto &brdf: BRDFs){

                            if(brdf.get_id() == new_rec.material.BRDF_id){
                                parser::Vec3f light_direction = normalize_vector(vector_subtract(light.get_position(), intersected_point));

                                brdf_result = brdf.get_all_result(new_rec.kd, new_rec.ks, normal, light_direction,
                                                                  camera_position, intersected_point,
                                                                  new_rec.material.refraction_index);

                            }

                        }

                        result = vector_add(result, get_brdf_result(light.get_position(), intersected_point, light.get_intensity(), brdf_result) );
                    }


                }

            }

            for(auto & light: spot_lights){


                bool in_shadow = point_in_shadow(time, near_distance, false, ignore_np,gaze,intersected_point, normal, shadow_ray_epsilon, light.get_position(),
                                                 shapes, bvh, bvh_start_index, blas_hiers,
                                                 tlas_counts, primitives_inside_blas,spheres, triangles, meshes);


                parser::Vec3f point_light_vector = normalize_vector(vector_subtract(light.get_position(), intersected_point));
                parser::Vec3f neg_spot_direction = normalize_vector(vector_multiply(light.get_direction(), -1));

                float angle_between_vectors = (acos(dot_product(point_light_vector, neg_spot_direction)));


                float coverage_angle_in_radians = light.get_coverage_angle() * M_PI /180;
                float falloff_angle_in_radians = light.get_falloff_angle() * M_PI /180;

                parser::Vec3f check_intensity{0,0,0};
                if(angle_between_vectors > (coverage_angle_in_radians)/2){

                    check_intensity = {0,0,0};

                }else if(angle_between_vectors < coverage_angle_in_radians / 2.f &&
                         angle_between_vectors > falloff_angle_in_radians / 2.f){


                    float s_hold = (cos(angle_between_vectors) - cos(coverage_angle_in_radians / 2.f))
                                   / (cos(falloff_angle_in_radians / 2.f) - cos(coverage_angle_in_radians / 2.f));
                    float fall_off_factor = pow(s_hold, 4);

                    check_intensity = vector_multiply(light.get_intensity() , fall_off_factor);

                }else{
                    check_intensity = light.get_intensity();
                }

                if(!in_shadow){


                    if(new_rec.material.BRDF_id == -1){
                        parser::Vec3f  diffuse_l = diffuse_light(normal, kd,
                                                                 light.get_position(), check_intensity,
                                                                 intersected_point);

                        parser::Vec3f specular_l = specular_light(normal, ks,
                                                                  new_rec.material.phong_exponent,light.get_position(),
                                                                  check_intensity, intersected_point, camera_position);


                        result.x += (diffuse_l.x + specular_l.x);
                        result.y += (diffuse_l.y + specular_l.y);
                        result.z += (diffuse_l.z + specular_l.z);
                    }else{
                        parser::Vec3f brdf_result{0,0,0};

                        //GET THE BRDF WITH THE CORRECT ID
                        for(auto &brdf: BRDFs){

                            if(brdf.get_id() == new_rec.material.BRDF_id){
                                parser::Vec3f light_direction = normalize_vector(vector_subtract(light.get_position(), intersected_point));

                                brdf_result = brdf.get_all_result(new_rec.kd, new_rec.ks, normal, light_direction,
                                                                  camera_position, intersected_point,
                                                                  new_rec.material.refraction_index);

                            }

                        }

                        result = vector_add(result, get_brdf_result(light.get_position(), intersected_point, light.get_intensity(), brdf_result) );

                    }


                }

            }


            for(auto & light: directional_lights){


                bool in_shadow = directional_light_point_in_shadow(time, near_distance, false, ignore_np, gaze,intersected_point,
                                                                   normal, shadow_ray_epsilon, light.get_direction(),
                                                                   shapes, bvh, bvh_start_index, blas_hiers,
                                                                   tlas_counts, primitives_inside_blas,spheres, triangles, meshes);

                if(!in_shadow){


                    if(new_rec.material.BRDF_id == -1){
                        parser::Vec3f  diffuse_l = diffuse_light_radiance(normal, kd,
                                                                          light.get_direction(), light.get_radiance());

                        parser::Vec3f specular_l = specular_light_radiance(normal, ks,
                                                                           new_rec.material.phong_exponent,light.get_direction(),
                                                                           light.get_radiance(), intersected_point, camera_position);


                        result.x += (diffuse_l.x + specular_l.x);
                        result.y += (diffuse_l.y + specular_l.y);
                        result.z += (diffuse_l.z + specular_l.z);
                    }else{

                    }


                }

            }

            for(auto & light: environment_lights){

                parser::Vec3f sample = get_random_rejection_sample(random_generator, normal);

                bool in_shadow = directional_light_point_in_shadow(time, near_distance, false,ignore_np,gaze,intersected_point, normal, shadow_ray_epsilon,
                                                        vector_multiply(sample,-1),
                                                                   shapes, bvh, bvh_start_index, blas_hiers,
                                                                   tlas_counts, primitives_inside_blas,spheres, triangles, meshes);

                parser::Vec3f radiance = vector_multiply(light.get_environment_luminance(sample),2*M_PI);
                if(!in_shadow){

                    if(new_rec.material.BRDF_id == -1){
                        parser::Vec3f diffuse_l = diffuse_light_radiance_env(normal, kd,
                                                                             sample, radiance);

                        parser::Vec3f specular_l = specular_light_radiance_env(normal, ks,
                                                                               new_rec.material.phong_exponent,
                                                                               sample,
                                                                               radiance, intersected_point,
                                                                               camera_position);


                        result.x += (diffuse_l.x + specular_l.x);
                        result.y += (diffuse_l.y + specular_l.y);
                        result.z += (diffuse_l.z + specular_l.z);
                    }else{

                    }


                }
            }


            int area_light_index = 0;
            for(auto & light: area_lights){
                parser::Vec3f sample_point = vector_add(light.position,
                                                        vector_add(vector_multiply(light.get_u(),
                                                                                   (area_samples[area_light_index].first-0.5)*light.get_size()),
                                                                   vector_multiply(light.get_v(),
                                                                                   (area_samples[area_light_index].second-0.5)*light.get_size())));

                bool in_shadow = point_in_shadow(time, near_distance, false, ignore_np,gaze,intersected_point, normal, shadow_ray_epsilon, sample_point,
                                                 shapes, bvh, bvh_start_index, blas_hiers,
                                                 tlas_counts, primitives_inside_blas,spheres, triangles, meshes);

                if(!in_shadow){

                    parser::Vec3f radiance = light.get_radiance(intersected_point, sample_point);

                    if(new_rec.material.BRDF_id == -1){
                        parser::Vec3f  diffuse_l = diffuse_light(normal, kd,
                                                                 sample_point, radiance,
                                                                 intersected_point);

                        parser::Vec3f specular_l = specular_light(normal, ks,
                                                                  new_rec.material.phong_exponent,sample_point, radiance,
                                                                  intersected_point, camera_position);


                        result.x += (diffuse_l.x + specular_l.x);
                        result.y += (diffuse_l.y + specular_l.y);
                        result.z += (diffuse_l.z + specular_l.z);
                    }else{
                        parser::Vec3f brdf_result{0,0,0};

                        //GET THE BRDF WITH THE CORRECT ID
                        for(auto &brdf: BRDFs){

                            if(brdf.get_id() == new_rec.material.BRDF_id){
                                parser::Vec3f light_direction = normalize_vector(vector_subtract(sample_point, intersected_point));

                                brdf_result = brdf.get_all_result(new_rec.kd, new_rec.ks, normal, light_direction,
                                                                  camera_position, intersected_point,
                                                                  new_rec.material.refraction_index);

                            }

                        }

                        result = vector_add(result, get_brdf_result(sample_point, intersected_point, radiance, brdf_result) );

                    }

                }
                area_light_index++;

            }

            for(auto & mesh: meshes){

                if(mesh->get_is_light()){

                    std::uniform_real_distribution<> gNURandomDistribution(0.0f, 0.999999f);
                    float rand1 = gNURandomDistribution(random_generator);
                    float rand2 = gNURandomDistribution(random_generator);
                    float rand3 = gNURandomDistribution(random_generator);


                     parser::Vec3f sample_point= mesh->get_mesh_object_light_ray(rand1,rand2, rand3);


                     bool in_shadow = point_in_shadow_in_check(time, near_distance, false, ignore_np,gaze,intersected_point, normal, shadow_ray_epsilon,
                                                    sample_point,
                                                    shapes, bvh, bvh_start_index, blas_hiers,
                                                    tlas_counts, primitives_inside_blas,spheres, triangles, meshes);


                    if(!in_shadow){

                        parser::Vec3f radiance = mesh->get_radiance();

                        if(new_rec.material.BRDF_id == -1){
                            parser::Vec3f  diffuse_l = diffuse_light(normal, kd,
                                                                     sample_point, radiance,
                                                                     intersected_point);

                            parser::Vec3f specular_l = specular_light(normal, ks,
                                                                      new_rec.material.phong_exponent,sample_point, radiance,
                                                                      intersected_point, camera_position);


                            result.x += (diffuse_l.x + specular_l.x);
                            result.y += (diffuse_l.y + specular_l.y);
                            result.z += (diffuse_l.z + specular_l.z);
                        }else{
                            parser::Vec3f brdf_result{0,0,0};

                            //GET THE BRDF WITH THE CORRECT ID
                            for(auto &brdf: BRDFs){

                                if(brdf.get_id() == new_rec.material.BRDF_id){
                                    parser::Vec3f light_direction = normalize_vector(vector_subtract(sample_point, intersected_point));

                                    brdf_result = brdf.get_all_result(new_rec.kd, new_rec.ks, normal, light_direction,
                                                                      camera_position, intersected_point,
                                                                      new_rec.material.refraction_index);

                                }

                            }

                            result = vector_add(result, get_brdf_result(sample_point, intersected_point, radiance, brdf_result) );

                        }

                    }
                }


            }


            for(auto & sphere: spheres){

                if(sphere->get_is_light()){

                    std::uniform_real_distribution<> gNURandomDistribution(0.0f, 0.999999f);
                    float rand1 = gNURandomDistribution(random_generator);
                    float rand2 = gNURandomDistribution(random_generator);

                    parser::Vec3f sample_point= sphere->get_sphere_object_light_ray(intersected_point,
                                                                                    rand1,rand2);


                    bool in_shadow = point_in_shadow_in_check(time, near_distance, false, ignore_np,gaze,
                                                              intersected_point, normal, shadow_ray_epsilon,
                                                     sample_point,
                                                     shapes, bvh, bvh_start_index, blas_hiers,
                                                     tlas_counts, primitives_inside_blas,spheres, triangles, meshes);


                    if(!in_shadow){

                        parser::Vec3f radiance = sphere->get_radiance();

                        if(new_rec.material.BRDF_id == -1){
                            parser::Vec3f  diffuse_l = diffuse_light(normal, kd,
                                                                     sample_point, radiance,
                                                                     intersected_point);

                            parser::Vec3f specular_l = specular_light(normal, ks,
                                                                      new_rec.material.phong_exponent,sample_point, radiance,
                                                                      intersected_point, camera_position);


                            result.x += (diffuse_l.x + specular_l.x);
                            result.y += (diffuse_l.y + specular_l.y);
                            result.z += (diffuse_l.z + specular_l.z);
                        }else{
                            parser::Vec3f brdf_result{0,0,0};

                            //GET THE BRDF WITH THE CORRECT ID
                            for(auto &brdf: BRDFs){

                                if(brdf.get_id() == new_rec.material.BRDF_id){
                                    parser::Vec3f light_direction = normalize_vector(vector_subtract(sample_point, intersected_point));

                                    brdf_result = brdf.get_all_result(new_rec.kd, new_rec.ks, normal, light_direction,
                                                                      camera_position, intersected_point,
                                                                      new_rec.material.refraction_index);

                                }

                            }

                            result = vector_add(result, get_brdf_result(sample_point, intersected_point, radiance, brdf_result) );

                        }

                    }
                }


            }


            parser::Vec3f ambient_l = get_ambient_light(ka, ambient_light);

            result.x += ambient_l.x;
            result.y += ambient_l.y;
            result.z += ambient_l.z;
        }



    }else{

        if(environment_lights.size() == 0){
            result.x = background_color.x;
            result.y = background_color.y;
            result.z = background_color.z;
        }else{

            for(auto & light: environment_lights){
                parser::Vec3f vec = normalize_vector(ray_direction);
                result = light.get_environment_luminance(vec);
            }

        }


    }

    return result;

}

