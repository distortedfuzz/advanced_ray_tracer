#include <vector>
#include <memory>
#include "../math/math.h"
#include "path_tracing.h"
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



parser::Vec3f path_trace(std::vector<texture> &textures,std::mt19937 &random_generator,std::vector<std::pair<float,float>> &area_samples, parser::Vec3f camera_position,
                            parser::Vec3f &ray_direction, float time, float near_distance,bool is_camera, bool ignore_np,parser::Vec3f &gaze,
                            std::vector<point_light> &point_lights, std::vector<area_light> &area_lights,
                            std::vector<directional_light> &directional_lights, std::vector<spot_light> &spot_lights,
                            std::vector<environment_light> &environment_lights,std::vector<std::shared_ptr<shape>> &shapes,
                            parser::Vec3f background_color, int current_recursion_depth,int max_recursion_depth,
                            int min_recursion_depth,
                            float shadow_ray_epsilon, std::vector<parser::Vec3f> &triangle_normals,
                            bvh_node* bvh, int bvh_start_index,std::vector<bvh_node*> &blas_hiers, parser::Vec3i &tlas_counts,
                            std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas, std::vector<std::shared_ptr<sphere>> &spheres,
                            std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes,
                            std::vector<BRDF> &BRDFs, camera &cam, float &throughput, int &path_depth){

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
                                         is_light_holder, is_cloud_holder,hold_radiance,textures,hold_tex);


    parser::Vec3f intersected_point = new_rec.intersected_point;
    auto intersected_shape = new_rec.intersected_shape;
    parser::Vec3f normal = new_rec.normal;

    parser::Vec3f kd = new_rec.kd;
    parser::Vec3f ks = new_rec.ks;


    bool total_internal_reflection = false;

    if(intersection){
        if(new_rec.is_light){

            return new_rec.radiance;
        }



        if(new_rec.material.is_mirror){

            parser::Vec3f reflection_ray = vector_add(vector_multiply(normal, -2 * dot_product(normal, ray_direction)),
                                                      ray_direction);
            if(new_rec.material.roughness > 1e-6){

                reflection_ray = get_glossy_reflection_ray(random_generator,reflection_ray, new_rec.material.roughness);

            }
            parser::Vec3f normalized_reflection_ray = normalize_vector(reflection_ray);
            parser::Vec3f epsilon_point = vector_add(intersected_point, vector_multiply(normal, shadow_ray_epsilon));

            parser::Vec3f mirror_color = path_trace(textures,random_generator,area_samples,epsilon_point, normalized_reflection_ray, time,
                                                       near_distance,false,ignore_np,gaze,
                                                       point_lights, area_lights, directional_lights, spot_lights,
                                                       environment_lights,shapes,
                                                       background_color, current_recursion_depth,
                                                       max_recursion_depth,
                                                       min_recursion_depth,
                                                       shadow_ray_epsilon, triangle_normals, bvh,
                                                       bvh_start_index, blas_hiers,
                                                       tlas_counts,primitives_inside_blas, spheres,
                                                       triangles, meshes, BRDFs, cam, throughput, path_depth);


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




                    parser::Vec3f transmission_color = path_trace(textures,random_generator,area_samples,refraction_in_epsilon_point, normalized_dielectric_refraction_ray,
                                                                     time, near_distance,false, ignore_np,gaze,
                                                                     point_lights,area_lights, directional_lights,spot_lights,
                                                                     environment_lights,shapes, background_color,
                                                                     current_recursion_depth,
                                                                     max_recursion_depth,
                                                                     min_recursion_depth,
                                                                     shadow_ray_epsilon, triangle_normals, bvh,
                                                                     bvh_start_index, blas_hiers,
                                                                     tlas_counts,primitives_inside_blas, spheres, triangles,
                                                                     meshes, BRDFs, cam, throughput, path_depth);

                    result.x += (transmission_color.x * dielectric_fresnel_ratios.transmission_ratio * attenuation_loss.x);
                    result.y += (transmission_color.y * dielectric_fresnel_ratios.transmission_ratio * attenuation_loss.y);
                    result.z += (transmission_color.z * dielectric_fresnel_ratios.transmission_ratio * attenuation_loss.z);

                    parser::Vec3f reflection_epsilon_point = vector_add(intersected_point,
                                                                        vector_multiply(neg_normal, shadow_ray_epsilon));


                    parser::Vec3f reflection_color = path_trace(textures,random_generator,area_samples,reflection_epsilon_point, normalized_reflection_ray,time, near_distance,
                                                                   false, ignore_np,gaze,
                                                                   point_lights,area_lights, directional_lights,spot_lights,
                                                                   environment_lights, shapes, background_color,
                                                                   current_recursion_depth,
                                                                   max_recursion_depth,
                                                                   min_recursion_depth,
                                                                   shadow_ray_epsilon, triangle_normals, bvh,
                                                                   bvh_start_index,blas_hiers,
                                                                   tlas_counts,primitives_inside_blas, spheres,
                                                                   triangles, meshes, BRDFs, cam, throughput
                                                                   ,path_depth);

                    result.x += (reflection_color.x * dielectric_fresnel_ratios.reflection_ratio * attenuation_loss.x);
                    result.y += (reflection_color.y * dielectric_fresnel_ratios.reflection_ratio * attenuation_loss.y);
                    result.z += (reflection_color.z * dielectric_fresnel_ratios.reflection_ratio * attenuation_loss.z);




                }else{
                    total_internal_reflection = true;
                    parser::Vec3f reflection_epsilon_point = vector_add(intersected_point,
                                                                        vector_multiply(neg_normal, shadow_ray_epsilon));


                    parser::Vec3f reflection_color = path_trace(textures,random_generator,area_samples,reflection_epsilon_point, normalized_reflection_ray,time, near_distance,
                                                                   false, ignore_np, gaze,
                                                                   point_lights,area_lights,directional_lights,spot_lights,
                                                                   environment_lights, shapes, background_color,
                                                                   current_recursion_depth,
                                                                   max_recursion_depth,
                                                                   min_recursion_depth,
                                                                   shadow_ray_epsilon,triangle_normals, bvh,
                                                                   bvh_start_index,blas_hiers,
                                                                   tlas_counts,primitives_inside_blas, spheres,
                                                                   triangles, meshes, BRDFs, cam, throughput,
                                                                   path_depth);

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

                int current_path_depth = path_depth;
                std::uniform_real_distribution<> gNURandomDistribution(0.0f, 0.9999f);

                parser::Vec3f direct_light_result{0,0,0};

                //CHECK DIRECT MESH LIGHT HIT
                for(auto & mesh: meshes){

                    if(mesh->get_is_light()){

                        std::uniform_real_distribution<> gNURandomDistribution(0.0f, 0.999999f);
                        float rand1 = gNURandomDistribution(random_generator);
                        float rand2 = gNURandomDistribution(random_generator);
                        float rand3 = gNURandomDistribution(random_generator);

                        parser::Vec3f sample_point= mesh->get_mesh_object_light_ray(rand1,rand2, rand3);


                        bool in_shadow = point_in_shadow_in_check(time, near_distance, false, ignore_np,gaze,reflection_epsilon_point, normal, shadow_ray_epsilon,
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


                                direct_light_result.x += (diffuse_l.x + specular_l.x);
                                direct_light_result.y += (diffuse_l.y + specular_l.y);
                                direct_light_result.z += (diffuse_l.z + specular_l.z);
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

                                direct_light_result = vector_add(direct_light_result, get_brdf_result(
                                        sample_point, intersected_point, radiance, brdf_result) );

                            }

                        }
                    }
                }



                //CHECK DIRECT SPHERE LIGHT HIT
                for(auto & sphere: spheres){

                    if(sphere->get_is_light()){

                        std::uniform_real_distribution<> gNURandomDistribution(0.0f, 0.999999f);
                        float rand1 = gNURandomDistribution(random_generator);
                        float rand2 = gNURandomDistribution(random_generator);

                        parser::Vec3f sample_point= sphere->get_sphere_object_light_ray(intersected_point,
                                                                                        rand1,rand2);

                        bool in_shadow = point_in_shadow_in_check(time, near_distance, false, ignore_np,gaze,reflection_epsilon_point, normal, shadow_ray_epsilon,
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


                                direct_light_result.x += (diffuse_l.x + specular_l.x);
                                direct_light_result.y += (diffuse_l.y + specular_l.y);
                                direct_light_result.z += (diffuse_l.z + specular_l.z);
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

                                direct_light_result = vector_add(direct_light_result, get_brdf_result(
                                        sample_point, intersected_point, radiance, brdf_result) );

                            }

                        }
                    }


                }


                parser::Vec3f reflection_color = path_trace(textures,random_generator,area_samples,reflection_epsilon_point, normalized_reflection_ray,time, near_distance,
                                                               false, ignore_np, gaze,
                                                               point_lights,area_lights, directional_lights,spot_lights,
                                                               environment_lights, shapes, background_color,
                                                               current_recursion_depth,
                                                               max_recursion_depth,
                                                               min_recursion_depth,
                                                               shadow_ray_epsilon,triangle_normals, bvh,
                                                               bvh_start_index,blas_hiers,
                                                               tlas_counts,primitives_inside_blas, spheres,
                                                               triangles, meshes, BRDFs, cam, throughput,
                                                               path_depth);

                //result = vector_add(result, reflection_color);
                /*
                result = vector_add(result, vector_multiply(
                                vector_add(reflection_color, direct_light_result), 0.5f * dielectric_fresnel_ratios.reflection_ratio));
                */

                parser::Vec3f dielectric_refraction_ray = refraction_ray(ray_direction, normal, 1.0,new_rec.material.refraction_index);
                parser::Vec3f normalized_dielectric_refraction_ray = normalize_vector(dielectric_refraction_ray);

                if(new_rec.material.roughness > 1e-6){

                    normalized_dielectric_refraction_ray = get_glossy_reflection_ray(random_generator,normalized_dielectric_refraction_ray, new_rec.material.roughness);

                }

                parser::Vec3f refraction_in_epsilon_point = vector_add(intersected_point,
                                                                       vector_multiply(normal, -1 * shadow_ray_epsilon));


                parser::Vec3f refraction_color = path_trace(textures,random_generator,area_samples,refraction_in_epsilon_point, normalized_dielectric_refraction_ray,time,
                                                               near_distance,false, ignore_np, gaze,
                                                               point_lights, area_lights, directional_lights,spot_lights,
                                                               environment_lights, shapes, background_color, current_recursion_depth,
                                                               max_recursion_depth, min_recursion_depth,
                                                               shadow_ray_epsilon,triangle_normals, bvh,
                                                               bvh_start_index,blas_hiers,
                                                               tlas_counts,primitives_inside_blas, spheres,
                                                               triangles, meshes, BRDFs, cam, throughput,
                                                               path_depth);

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

            conductor_color = path_trace(textures,random_generator,area_samples,epsilon_point, normalized_reflection_ray,time,
                                            near_distance,false, ignore_np, gaze,
                                            point_lights,area_lights, directional_lights,spot_lights,
                                            environment_lights, shapes, background_color,
                                            current_recursion_depth,
                                            max_recursion_depth, min_recursion_depth,
                                            shadow_ray_epsilon, triangle_normals, bvh, bvh_start_index,blas_hiers,
                                            tlas_counts,primitives_inside_blas, spheres,
                                            triangles, meshes, BRDFs, cam, throughput,
                                            path_depth);

            result.x += (conductor_color.x * new_rec.material.mirror.x)*conductor_fresnel_ratios.reflection_ratio;
            result.y += (conductor_color.y * new_rec.material.mirror.y)*conductor_fresnel_ratios.reflection_ratio;
            result.z += (conductor_color.z * new_rec.material.mirror.z)*conductor_fresnel_ratios.reflection_ratio;

        }

        if(!total_internal_reflection){
            path_depth++;
            int path_depth_now = path_depth;
            //NO NEXT EVENT ESTIMATION*************************************************************************************************************************
            if(!cam.get_path_tracing_params()[1]){
                std::uniform_real_distribution<> gNURandomDistribution(0.0f, 0.9999f);

                parser::Vec3f eps_point = vector_add(intersected_point, vector_multiply(normal, 1e-3));


                parser::Vec3f radiance{0,0,0};
                parser::Vec3f brdf_result{0,0,0};
                int splitting_factor = cam.get_splitting_factor();
                if(splitting_factor != 1 && is_camera){

                    parser::Vec3f total_rad{0,0,0};
                    for(int o = 0 ; o < splitting_factor; o++){
                        float rand1 = gNURandomDistribution(random_generator);
                        float rand2 = gNURandomDistribution(random_generator);

                        parser::Vec3f path_sample{0,0,0};
                        if(cam.get_path_tracing_params()[0]){
                            path_sample = get_cos_importance_sample(normal, rand1, rand2);
                        }else{
                            path_sample = get_uniform_sample(normal, rand1, rand2);
                        }


                        //GET THE BRDF WITH THE CORRECT ID
                        for(auto &brdf: BRDFs){

                            if(brdf.get_id() == new_rec.material.BRDF_id){
                                parser::Vec3f light_direction = normalize_vector(path_sample);

                                brdf_result = brdf.get_all_result(new_rec.kd, new_rec.ks, normal, light_direction,
                                                                  camera_position, intersected_point,
                                                                  new_rec.material.refraction_index);

                            }

                        }

                        float through_change = get_vector_magnitude(get_brdf_result_new(radiance, brdf_result));
                        throughput -= through_change;

                        parser::Vec3f single_rad = path_trace(textures,random_generator,area_samples,eps_point,
                                                              path_sample,time,
                                                              near_distance,false, ignore_np, gaze,
                                                              point_lights,area_lights, directional_lights,spot_lights,
                                                              environment_lights, shapes, background_color,
                                                              current_recursion_depth,
                                                              max_recursion_depth, min_recursion_depth,
                                                              shadow_ray_epsilon, triangle_normals, bvh, bvh_start_index,blas_hiers,
                                                              tlas_counts,primitives_inside_blas, spheres,
                                                              triangles, meshes, BRDFs, cam, throughput,
                                                              path_depth);

                        path_depth = path_depth_now;
                        if(cam.get_path_tracing_params()[0]){
                            single_rad = vector_multiply(single_rad, M_PI /
                                                                     cos(asin(sqrt(rand2))));
                        }else{
                            single_rad = vector_multiply(single_rad, 2*M_PI);
                        }

                        total_rad = vector_add(total_rad, single_rad);
                    }

                    radiance = vector_multiply(total_rad, 1.0f / splitting_factor);

                }else{

                    float rand1 = gNURandomDistribution(random_generator);
                    float rand2 = gNURandomDistribution(random_generator);

                    parser::Vec3f path_sample{0,0,0};
                    if(cam.get_path_tracing_params()[0]){
                        path_sample = get_cos_importance_sample(normal, rand1, rand2);
                    }else{
                        path_sample = get_uniform_sample(normal, rand1, rand2);
                    }



                    //GET THE BRDF WITH THE CORRECT ID
                    for(auto &brdf: BRDFs){

                        if(brdf.get_id() == new_rec.material.BRDF_id){
                            parser::Vec3f light_direction = normalize_vector(path_sample);

                            brdf_result = brdf.get_all_result(new_rec.kd, new_rec.ks, normal, light_direction,
                                                              camera_position, intersected_point,
                                                              new_rec.material.refraction_index);

                        }

                    }


                    //USE BRDF RESULT FOR ROULETTE
                    if(cam.get_path_tracing_params()[2] && current_recursion_depth >= min_recursion_depth){
                        std::uniform_real_distribution<> RouletteRandomDistribution(0.0f, 1.0f);
                        float roulette_rand = RouletteRandomDistribution(random_generator);

                        if(roulette_rand > throughput){
                            return parser::Vec3f{0,0,0};
                        }

                    }

                    float through_change = get_vector_magnitude(brdf_result);
                    throughput -= through_change;

                    radiance = path_trace(textures,random_generator,area_samples,eps_point,
                                          path_sample,time,
                                          near_distance,false, ignore_np, gaze,
                                          point_lights,area_lights, directional_lights,spot_lights,
                                          environment_lights, shapes, background_color,
                                          current_recursion_depth,
                                          max_recursion_depth, min_recursion_depth,
                                          shadow_ray_epsilon, triangle_normals, bvh, bvh_start_index,blas_hiers,
                                          tlas_counts,primitives_inside_blas, spheres,
                                          triangles, meshes, BRDFs, cam, throughput,
                                          path_depth);

                    if(cam.get_path_tracing_params()[0]){
                        radiance = vector_multiply(radiance, M_PI /
                                                             cos(asin(sqrt(rand2))));
                    }else{
                        radiance = vector_multiply(radiance, 2*M_PI);
                    }
                }



                result = vector_add(result, get_brdf_result_new(radiance, brdf_result));


            //!!!!!NEXT EVENT ESTIMATION*************************************************************************************************************************
            }else{
                int current_path_depth = path_depth;
                std::uniform_real_distribution<> gNURandomDistribution(0.0f, 0.9999f);

                parser::Vec3f eps_point = vector_add(intersected_point, vector_multiply(normal, 1e-6));

                parser::Vec3f direct_light_result{0,0,0};

                //CHECK DIRECT MESH LIGHT HIT
                for(auto & mesh: meshes){

                    if(mesh->get_is_light()){

                        std::uniform_real_distribution<> gNURandomDistribution(0.0f, 0.999999f);
                        float rand1 = gNURandomDistribution(random_generator);
                        float rand2 = gNURandomDistribution(random_generator);
                        float rand3 = gNURandomDistribution(random_generator);

                        parser::Vec3f sample_point= mesh->get_mesh_object_light_ray(rand1,rand2, rand3);

                        hit_record path_rec{-1, 0.0};
                        Eigen::Matrix4f path_starting_matrix = Eigen::Matrix4f::Identity();
                        parser::Material path_hold_mat;
                        bool path_is_light_holder = false;
                        bool path_is_cloud_holder = false;
                        parser::Vec3f path_hold_radiance;
                        std::vector<int> path_hold_tex;
                        parser::Vec3f refrac_ray = normalize_vector(vector_subtract(sample_point, eps_point));

                        bool path_intersection = bvh_intersect_mb(time,near_distance, is_camera, ignore_np, gaze,bvh, eps_point,refrac_ray ,path_rec,
                                                                  shapes, blas_hiers,
                                                                  tlas_counts,primitives_inside_blas, path_starting_matrix, spheres, triangles,
                                                                  meshes, path_hold_mat,
                                                                  path_is_light_holder, path_is_cloud_holder,
                                                                  path_hold_radiance,textures,path_hold_tex);

                        if(path_intersection && path_rec.material.is_dielectric){
                            direct_light_result = parser::Vec3f{0,0,0};
                            continue;
                        }


                        bool in_shadow = point_in_shadow_in_check(time, near_distance, false, ignore_np,gaze,eps_point, normal, shadow_ray_epsilon,
                                                                  sample_point,
                                                                  shapes, bvh, bvh_start_index, blas_hiers,
                                                                  tlas_counts, primitives_inside_blas,spheres, triangles, meshes);


                        if(!in_shadow){

                            parser::Vec3f radiance = mesh->get_radiance();
                            float r_distance = get_vector_magnitude(vector_subtract(sample_point, intersected_point));
                            float cos_theta_i = dot_product(normal, normalize_vector(vector_subtract(sample_point, intersected_point)));

                            //float probability = (r_distance * r_distance) / (mesh->total_mesh_area * cos_theta_i);

                            radiance = vector_multiply(radiance, mesh->total_mesh_area * cos_theta_i);


                            if(new_rec.material.BRDF_id == -1){
                                parser::Vec3f  diffuse_l = diffuse_light(normal, kd,
                                                                         sample_point, radiance,
                                                                         intersected_point);

                                parser::Vec3f specular_l = specular_light(normal, ks,
                                                                          new_rec.material.phong_exponent,sample_point, radiance,
                                                                          intersected_point, camera_position);


                                direct_light_result.x += (diffuse_l.x + specular_l.x);
                                direct_light_result.y += (diffuse_l.y + specular_l.y);
                                direct_light_result.z += (diffuse_l.z + specular_l.z);
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

                                direct_light_result = vector_add(direct_light_result, get_brdf_result(
                                        sample_point, intersected_point, radiance, brdf_result) );

                            }

                        }
                    }
                }



                //CHECK DIRECT SPHERE LIGHT HIT
                for(auto & sphere: spheres){

                    if(sphere->get_is_light()){

                        std::uniform_real_distribution<> gNURandomDistribution(0.0f, 0.999999f);
                        float rand1 = gNURandomDistribution(random_generator);
                        float rand2 = gNURandomDistribution(random_generator);

                        parser::Vec3f sample_point= sphere->get_sphere_object_light_ray(intersected_point,
                                                                                        rand1,rand2);


                        hit_record path_rec{-1, 0.0};
                        Eigen::Matrix4f path_starting_matrix = Eigen::Matrix4f::Identity();
                        parser::Material path_hold_mat;
                        bool path_is_light_holder = false;
                        bool path_is_cloud_holder = false;
                        parser::Vec3f path_hold_radiance;
                        std::vector<int> path_hold_tex;
                        parser::Vec3f refrac_ray = normalize_vector(vector_subtract(sample_point, eps_point));

                        bool path_intersection = bvh_intersect_mb(time,near_distance, is_camera, ignore_np, gaze,bvh, eps_point,refrac_ray ,path_rec,
                                                             shapes, blas_hiers,
                                                             tlas_counts,primitives_inside_blas, path_starting_matrix, spheres, triangles,
                                                             meshes, path_hold_mat,
                                                             path_is_light_holder, path_is_cloud_holder,
                                                             path_hold_radiance,textures,path_hold_tex);

                        if(path_intersection && path_rec.material.is_dielectric){
                            direct_light_result = parser::Vec3f{0,0,0};
                            continue;
                        }



                        bool in_shadow = point_in_shadow_in_check(time, near_distance, false, ignore_np,gaze,eps_point, normal, shadow_ray_epsilon,
                                                                  sample_point,
                                                                  shapes, bvh, bvh_start_index, blas_hiers,
                                                                  tlas_counts, primitives_inside_blas,spheres, triangles, meshes);


                        if(!in_shadow){

                            parser::Vec3f radiance = sphere->get_radiance();

                            float cos_theta_max = sphere->get_cos_theta_max(intersected_point);
                            float cos_theta_i = dot_product(normal, normalize_vector(vector_subtract(sample_point, intersected_point)));
                            // Compute vector to sphere center
                            parser::Vec3f center_vector = vector_subtract(sphere->get_center(), intersected_point);
                            float dist = get_vector_magnitude(center_vector);

                            // Compute sin(theta_max) with safeguards
                            float sin_max = std::fmin(sphere->get_radius() / (dist + 1e-6f), 1.0f);
                            radiance = vector_multiply(radiance, sin_max/(15 * 2 * M_PI *(1-cos_theta_max)));
                            if(new_rec.material.BRDF_id == -1){
                                parser::Vec3f  diffuse_l = diffuse_light(normal, kd,
                                                                         sample_point, radiance,
                                                                         intersected_point);

                                parser::Vec3f specular_l = specular_light(normal, ks,
                                                                          new_rec.material.phong_exponent,sample_point, radiance,
                                                                          intersected_point, camera_position);


                                direct_light_result.x += (diffuse_l.x + specular_l.x);
                                direct_light_result.y += (diffuse_l.y + specular_l.y);
                                direct_light_result.z += (diffuse_l.z + specular_l.z);
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

                                direct_light_result = vector_add(direct_light_result, get_brdf_result(
                                                        sample_point, intersected_point, radiance, brdf_result) );

                            }

                        }
                    }


                }



                bool refraction_hit = false;
                parser::Vec3f radiance{0,0,0};
                parser::Vec3f brdf_result{0,0,0};
                int splitting_factor = cam.get_splitting_factor();
                if(splitting_factor != 1 && is_camera){

                    parser::Vec3f total_rad{0,0,0};
                    for(int o = 0 ; o < splitting_factor; o++){
                        float rand1 = gNURandomDistribution(random_generator);
                        float rand2 = gNURandomDistribution(random_generator);

                        parser::Vec3f path_sample{0,0,0};
                        if(cam.get_path_tracing_params()[0]){
                            path_sample = get_cos_importance_sample(normal, rand1, rand2);
                        }else{
                            path_sample = get_uniform_sample(normal, rand1, rand2);
                        }


                        //GET THE BRDF WITH THE CORRECT ID
                        for(auto &brdf: BRDFs){

                            if(brdf.get_id() == new_rec.material.BRDF_id){
                                parser::Vec3f light_direction = normalize_vector(path_sample);

                                brdf_result = brdf.get_all_result(new_rec.kd, new_rec.ks, normal, light_direction,
                                                                  camera_position, intersected_point,
                                                                  new_rec.material.refraction_index);

                            }

                        }

                        float through_change = get_vector_magnitude(get_brdf_result_new(radiance, brdf_result));
                        throughput -= through_change;

                        parser::Vec3f single_rad = path_trace(textures,random_generator,area_samples,eps_point,
                                                              path_sample,time,
                                                              near_distance,false, ignore_np, gaze,
                                                              point_lights,area_lights, directional_lights,spot_lights,
                                                              environment_lights, shapes, background_color,
                                                              current_recursion_depth,
                                                              max_recursion_depth, min_recursion_depth,
                                                              shadow_ray_epsilon, triangle_normals, bvh, bvh_start_index,blas_hiers,
                                                              tlas_counts,primitives_inside_blas, spheres,
                                                              triangles, meshes, BRDFs, cam, throughput,
                                                              path_depth);

                        if(cam.get_path_tracing_params()[0]){
                            single_rad = vector_multiply(single_rad, M_PI /
                                                                     cos(asin(sqrt(rand2))));
                        }else{
                            single_rad = vector_multiply(single_rad, 2*M_PI);
                        }

                        total_rad = vector_add(total_rad, single_rad);
                    }

                    radiance = vector_multiply(total_rad, 1.0f / splitting_factor);

                }else{

                    float rand1 = gNURandomDistribution(random_generator);
                    float rand2 = gNURandomDistribution(random_generator);

                    parser::Vec3f path_sample{0,0,0};
                    if(cam.get_path_tracing_params()[0]){
                        path_sample = get_cos_importance_sample(normal, rand1, rand2);
                    }else{
                        path_sample = get_uniform_sample(normal, rand1, rand2);
                    }

                    hit_record path_rec{-1, 0.0};
                    Eigen::Matrix4f path_starting_matrix = Eigen::Matrix4f::Identity();
                    parser::Material path_hold_mat;
                    bool path_is_light_holder = false;
                    bool path_is_cloud_holder = false;
                    parser::Vec3f path_hold_radiance;
                    std::vector<int> path_hold_tex;

                    bool path_intersection = bvh_intersect_mb(time,near_distance, is_camera, ignore_np, gaze,bvh, eps_point,path_sample ,path_rec,
                                                              shapes, blas_hiers,
                                                              tlas_counts,primitives_inside_blas, path_starting_matrix, spheres, triangles,
                                                              meshes, path_hold_mat,
                                                              path_is_light_holder,path_is_cloud_holder,
                                                              path_hold_radiance,textures,path_hold_tex);

                    if(path_intersection && path_rec.material.is_dielectric){
                        refraction_hit = true;
                    }

                    //GET THE BRDF WITH THE CORRECT ID
                    for(auto &brdf: BRDFs){

                        if(brdf.get_id() == new_rec.material.BRDF_id){
                            parser::Vec3f light_direction = normalize_vector(path_sample);

                            brdf_result = brdf.get_all_result(new_rec.kd, new_rec.ks, normal, light_direction,
                                                              camera_position, intersected_point,
                                                              new_rec.material.refraction_index);

                        }

                    }


                    //USE BRDF RESULT FOR ROULETTE
                    if(cam.get_path_tracing_params()[2] && current_recursion_depth >= min_recursion_depth){
                        std::uniform_real_distribution<> RouletteRandomDistribution(0.0f, 1.0f);
                        float roulette_rand = RouletteRandomDistribution(random_generator);

                        if(roulette_rand > throughput){
                            return parser::Vec3f{0,0,0};
                        }

                    }

                    float through_change = get_vector_magnitude(brdf_result);
                    throughput -= through_change;

                    radiance = path_trace(textures,random_generator,area_samples,eps_point,
                                          path_sample,time,
                                          near_distance,false, ignore_np, gaze,
                                          point_lights,area_lights, directional_lights,spot_lights,
                                          environment_lights, shapes, background_color,
                                          current_recursion_depth,
                                          max_recursion_depth, min_recursion_depth,
                                          shadow_ray_epsilon, triangle_normals, bvh, bvh_start_index,blas_hiers,
                                          tlas_counts,primitives_inside_blas, spheres,
                                          triangles, meshes, BRDFs, cam, throughput,
                                          path_depth);

                    if(cam.get_path_tracing_params()[0]){
                        radiance = vector_multiply(radiance, M_PI /
                                                             cos(asin(sqrt(rand2))));
                    }else{
                        radiance = vector_multiply(radiance, 2*M_PI);
                    }
                }



                result = vector_add(result, get_brdf_result_new(radiance, brdf_result));



                //DIRECT HIT INDIRECT NO HIT
                if(get_vector_magnitude(direct_light_result) > 0 && get_vector_magnitude(result) > 0 &&
                   path_depth > current_path_depth + 1){

                    return vector_add(direct_light_result, result);

                    //DIRECT HIT INDIRECT HIT AT THE SAME TIME -> X/2
                }else if(get_vector_magnitude(direct_light_result) > 0 && get_vector_magnitude(result) > 0 &&
                         path_depth == current_path_depth + 1){

                    if(refraction_hit){
                        return vector_multiply(vector_add(direct_light_result, result), 1.0f);
                    }else{
                        return vector_multiply(vector_add(direct_light_result, result), 0.5f);
                    }


                    //NO DIRECT HIT
                }else if(get_vector_magnitude(direct_light_result) <= 0.0f){
                    return result;

                }else if(get_vector_magnitude(result) <= 0.0f){
                    return direct_light_result;
                }

                return parser::Vec3f{0,0,0};

            }

            return result;

        }



    }else{

        result.x = background_color.x;
        result.y = background_color.y;
        result.z = background_color.z;

    }

    return result;

}