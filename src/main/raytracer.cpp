#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <atomic>
#include <thread>
#include <memory>
#include <algorithm>
#include "../math/math.h"
#include "../camera/camera.h"
#include "../lights/point_light.h"
#include "../lights/directional_light.h"
#include "../lights/spot_light.h"
#include "../lights/environment_light.h"
#include "../tonemapping/tonemapping.h"
#include "../shapes/mesh.h"
#include "../shading/BRDF.h"
#include "path_tracing.h"
#include "color.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../write/stb_image_write.h"
#include "BVH.h"
#include <random>

#define TINYEXR_USE_MINIZ 0
#include <zlib.h>
#define TINYEXR_IMPLEMENTATION
#include "../parser/tinyexr.h"


typedef unsigned char RGB[3];



void save_exr_image(std::vector<float>& pixel_data, int width, int height, const char* filename) {
    // Ensure pixel_data is correct size
    if (pixel_data.size() != width * height * 3) {
        std::cerr << "Error: pixel_data size mismatch." << std::endl;
        return;
    }

    EXRHeader header;
    InitEXRHeader(&header);

    EXRImage image;
    InitEXRImage(&image);

    image.num_channels = 3;

    // Split the interleaved RGB pixel data into separate channels
    std::vector<float> red_channel(width * height);
    std::vector<float> green_channel(width * height);
    std::vector<float> blue_channel(width * height);

    for (int i = 0; i < width * height; i++) {
        red_channel[i] = pixel_data[i * 3 + 0];
        green_channel[i] = pixel_data[i * 3 + 1];
        blue_channel[i] = pixel_data[i * 3 + 2];
    }

    float* image_ptrs[3] = {red_channel.data(), green_channel.data(), blue_channel.data()};
    image.images = (unsigned char**)image_ptrs;
    image.width = width;
    image.height = height;

    // Set up channel information in the header
    header.num_channels = 3;
    header.channels = (EXRChannelInfo*)malloc(sizeof(EXRChannelInfo) * header.num_channels);
    strncpy(header.channels[0].name, "R", 255);
    strncpy(header.channels[1].name, "G", 255);
    strncpy(header.channels[2].name, "B", 255);

    // Specify pixel types and requested pixel types
    header.pixel_types = (int*)malloc(sizeof(int) * header.num_channels);
    header.requested_pixel_types = (int*)malloc(sizeof(int) * header.num_channels);
    for (int i = 0; i < header.num_channels; i++) {
        header.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;          // Input pixel type
        header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_HALF; // Output pixel type
    }

    const char* err = nullptr;
    int ret = SaveEXRImageToFile(&image, &header, filename, &err);
    if (ret != TINYEXR_SUCCESS) {
        std::cerr << "Error saving EXR: " << (err ? err : "Unknown error") << std::endl;
        FreeEXRErrorMessage(err);
    } else {
        std::cout << "Saved EXR file: " << filename << std::endl;
    }

    // Free allocated memory in the header
    free(header.channels);
    free(header.pixel_types);
    free(header.requested_pixel_types);
}


int main(int argc, char* argv[])
{
    auto start = std::chrono::high_resolution_clock::now();
    parser::Scene scene;


    std::string input_file = "../inputs/";
    std::string input_xml = argv[1];
    std::string all_input_xml = input_file + input_xml;
    std::cout<<all_input_xml<<std::endl;


    scene.loadFromXml(all_input_xml);

    std::cout<<"load completed"<<std::endl;

    std::vector<camera> cameras;
    int cam_id = 0;
    for(auto &cam : scene.cameras){
        cam_id++;


        camera new_camera(cam_id, cam.is_look_at,cam.position, cam.gaze, cam.gaze_point, cam.up, cam.near_plane,
                           cam.fov_y,cam.near_distance, cam.image_height, cam.image_width, cam.num_samples,cam.image_name,
                           cam.focus_distance, cam.aperture_size, cam.transformations,
                           scene.translations, scene.rotations, scene.scalings, scene.composites, cam.tone_map, cam.handedness, cam.ignore_np,
                           cam.path_tracing, cam.path_tracing_params, cam.splitting_factor);

        cameras.push_back(new_camera);
    }


    std::vector<point_light> point_lights;
    int light_id = 0;
    for(auto &p_light : scene.point_lights){
        light_id++;
        point_light new_light(light_id, p_light.position, p_light.intensity, p_light.transformations, scene.translations, scene.rotations,scene.scalings,scene.composites);

        point_lights.push_back(new_light);
    }


    std::vector<area_light> area_lights;
    light_id = 0;
    for(auto &a_light : scene.area_lights){
        light_id++;
        area_light new_light(light_id, a_light.size, a_light.position, a_light.normal, a_light.radiance);

        area_lights.push_back(new_light);
    }


    std::vector<directional_light> directional_lights;
    light_id = 0;
    for(auto &d_light : scene.directional_lights){
        light_id++;
        directional_light new_light(light_id, d_light.direction, d_light.radiance);

        directional_lights.push_back(new_light);
    }


    std::vector<spot_light> spot_lights;
    light_id = 0;
    for(auto &s_light : scene.spot_lights){
        light_id++;
        spot_light new_light(light_id, s_light.position, s_light.direction, s_light.intensity, s_light.coverage_angle, s_light.falloff_angle);

        spot_lights.push_back(new_light);
    }

    std::vector<environment_light> environment_lights;
    for(auto &e_light : scene.environment_lights){
        light_id++;
        environment_light new_light(e_light.id, e_light.type, e_light.image_id, scene.images);

        environment_lights.push_back(new_light);
    }

    //**********************************************************************************************************************************************
    //BRDFs
    std::vector<BRDF> brdfs;
    for(auto &brdf: scene.brdfs){
        BRDF new_brdf(brdf.mode, brdf.id, brdf.normalized, brdf.exponent, brdf.kdfresnel);

        brdfs.push_back(new_brdf);
    }

    //**********************************************************************************************************************************************
    //textures
    std::vector<texture> textures;
    for(auto &tex: scene.textures){
        texture new_texture(tex.image_id, tex.normalizer, scene.images, tex.decal_mode, tex.interpolation, tex.noise_conversion,
                            tex.bump_factor, tex.noise_scale, tex.num_octaves, tex.texture_type, tex.scale, tex.offset, tex.black_color, tex.white_color);
        textures.push_back(new_texture);
    }

    std::cout<<"texture size:"<<textures.size()<<std::endl;

    //********************************************************************************************************************************************************
    //objects
    //want to create blas for each of them, i do not care which type

    //each local object with or without transformation or instancing
    std::vector<bvh_node*> blas_hiers;
    //holds list of all primitives inside a single object as a list
    std::vector<std::vector<std::shared_ptr<shape>>> blas_primitives;
    //top level bboxes for objects
    std::vector<std::shared_ptr<bbox>> tlas_bboxes;

    //motion blurs, kept as translations
    std::vector<translation> motion_blurs;
    std::vector<int> motion_blur_tlas_id;

    //Sphere start********************************************
    std::cout<<"start putting spheres"<<std::endl;

    std::vector<std::shared_ptr<sphere>> spheres;
    int sphere_id = 0;
    int iterator = 0;
    int motion_blur_iterator = 0;
    //get spheres and add them to the spheres matrix
    for(auto &sph : scene.spheres){


        //actual creation
        auto new_sphere = std::make_shared<sphere>(sphere_id, sph.material_id,scene.materials, sph.texture_ids,
                                                   sph.center_vertex_id,sph.radius, scene.vertex_data,sph.transformations,
                                                   scene.translations, scene.rotations, scene.scalings, scene.composites,
                                                   sph.radiance, sph.is_light, sph.is_cloud);

        spheres.push_back(new_sphere);


        //blas
        std::vector<std::shared_ptr<shape>> sphere_holder;
        sphere_holder.push_back(new_sphere);
        int count = 0;
        bvh_node* bvh = build_bvh_blas(sphere_holder, 0, 1, count);
        bvh->primitive_id = -1;

        blas_hiers.push_back(bvh);

        blas_primitives.push_back(sphere_holder);

        sphere_holder.clear();



        Eigen::Matrix4f tr_matrix = Eigen::Matrix4f::Identity();
        std::shared_ptr<bbox> new_bbox;
        //tlas
        if(new_sphere->get_transformation_matrix() == Eigen::Matrix4f::Identity()){
            sphere_holder.push_back(new_sphere);
            new_bbox = std::make_shared<bbox>(sphere_holder, 0, 1, true, 0, iterator, scene.materials[sph.material_id-1],
                                              sph.is_light, sph.is_cloud, sph.radiance,sph.texture_ids);

        }else{
            parser::Vec3f center = new_sphere->get_center();
            float radius = new_sphere->get_radius();

            std::vector<parser::Transformation> transformations = new_sphere->get_transformations();
            tr_matrix = new_sphere->get_transformation_matrix();

            auto new_sph = std::make_shared<sphere>(center,radius, transformations, scene.translations,
                                                    scene.rotations,scene.scalings, scene.composites,sph.material_id, scene.materials,
                                                    sph.texture_ids,sph.radiance, sph.is_light, sph.is_cloud);

            new_bbox = std::make_shared<bbox>(sphere_holder, 0, 1, true, 0, iterator, scene.materials[sph.material_id-1],
                                              sph.is_light, sph.is_cloud, sph.radiance, sph.texture_ids);
            new_bbox->apply_transformation(tr_matrix);
            new_bbox->transformation_matrix = tr_matrix;

        }

        if(sph.motion_blur.x != 0 || sph.motion_blur.y != 0 || sph.motion_blur.z != 0){
            //motion blur end bbox
            parser::Translation mot_blur;
            mot_blur.id = scene.translations.size();
            mot_blur.translation_x = sph.motion_blur.x;
            mot_blur.translation_y = sph.motion_blur.y;
            mot_blur.translation_z = sph.motion_blur.z;



            scene.translations.push_back(mot_blur);

            parser::Transformation new_mot_blur;
            new_mot_blur.transformation_type = 0;
            new_mot_blur.id = mot_blur.id +1;
            std::vector<parser::Transformation> total_transformations = sph.transformations;

            total_transformations.insert(total_transformations.begin(),new_mot_blur);

            Eigen::Matrix4f tr_matrix_end = calculate_transformation_matrix(total_transformations, scene.translations, scene.rotations, scene.scalings, scene.composites);
            auto end_bbox = std::make_shared<bbox>(sphere_holder, 0,1,
                                                   true, 0,iterator, scene.materials[sph.material_id-1],
                                                   sph.is_light, sph.is_cloud, sph.radiance,sph.texture_ids);

            end_bbox->apply_transformation(tr_matrix_end);
            end_bbox->transformation_matrix = tr_matrix_end;


            std::vector<parser::Transformation> blur_vec;
            blur_vec.push_back(new_mot_blur);

            Eigen::Matrix4f motion_blur_matrix_calc = calculate_transformation_matrix(blur_vec, scene.translations, scene.rotations, scene.scalings,scene.composites);

            //combine
            auto combined_bbox = std::make_shared<bbox>(motion_blur_matrix_calc,new_bbox, end_bbox,
                                                        true, 0,  iterator, scene.materials[sph.material_id-1],
                                                        sph.is_light, sph.is_cloud, sph.radiance, sph.texture_ids);


            combined_bbox->transformation_matrix = tr_matrix;

            tlas_bboxes.push_back(combined_bbox);
        }else{
            tlas_bboxes.push_back(new_bbox);
        }

        iterator++;
        motion_blur_iterator++;
        sphere_id++;
        sphere_holder.clear();

    }


    //Triangle start********************************************

    std::cout<<"start putting_triangles"<<std::endl;
    std::vector<std::shared_ptr<triangle>> triangles;
    int triangle_id = 0;

    for(auto &tri : scene.triangles){

        parser::Vec3i indices;
        indices.x = tri.indices.v0_id;
        indices.y = tri.indices.v1_id;
        indices.z = tri.indices.v2_id;

        auto new_triangle = std::make_shared<triangle>(triangle_id, tri.material_id, tri.texture_ids,scene.materials,
                                                       indices, scene.tex_coord_data, scene.vertex_data,
                                                       tri.transformations, scene.translations, scene.rotations, scene.scalings,scene.composites,0,0,
                                                       tri.radiance, tri.is_light, false);

        triangles.push_back(new_triangle);

        //blas
        std::vector<std::shared_ptr<shape>> triangle_holder;
        triangle_holder.push_back(new_triangle);
        int count = 0;
        bvh_node* bvh = build_bvh_blas(triangle_holder, 0, 1, count);
        bvh->primitive_id = -1;

        blas_hiers.push_back(bvh);
        blas_primitives.push_back(triangle_holder);
        triangle_holder.clear();


        Eigen::Matrix4f tr_matrix = Eigen::Matrix4f::Identity();
        std::shared_ptr<bbox> new_bbox;
        //tlas
        if(new_triangle->get_transformation_matrix() == Eigen::Matrix4f::Identity()){
            triangle_holder.push_back(new_triangle);
            new_bbox = std::make_shared<bbox>(triangle_holder, 0, 1, true, 1, iterator, scene.materials[tri.material_id-1],
                                              tri.is_light, false, tri.radiance, tri.texture_ids);

        }else{
            std::vector<parser::Vec3f> corners = new_triangle->get_corners();
            tr_matrix = new_triangle->get_transformation_matrix();

            auto new_tri = std::make_shared<triangle>(corners, tr_matrix, tri.material_id, tri.texture_ids,scene.materials,
                                                      tri.radiance, tri.is_light, false);
            triangle_holder.push_back(new_tri);

            new_bbox = std::make_shared<bbox>(triangle_holder, 0, 1, true, 1, iterator, scene.materials[tri.material_id-1],
                                              tri.is_light,false, tri.radiance, tri.texture_ids);
            new_bbox->apply_transformation(tr_matrix);
            new_bbox->transformation_matrix = tr_matrix;
        }

        if(tri.motion_blur.x != 0 || tri.motion_blur.y != 0 || tri.motion_blur.z != 0){
            //motion blur end bbox
            parser::Translation mot_blur;
            mot_blur.id = scene.translations.size();
            mot_blur.translation_x = tri.motion_blur.x;
            mot_blur.translation_y = tri.motion_blur.y;
            mot_blur.translation_z = tri.motion_blur.z;

            scene.translations.push_back(mot_blur);

            parser::Transformation new_mot_blur;
            new_mot_blur.transformation_type = 0;
            new_mot_blur.id = mot_blur.id +1;
            std::vector<parser::Transformation> total_transformations = tri.transformations;

            total_transformations.insert(total_transformations.begin(),new_mot_blur);

            Eigen::Matrix4f tr_matrix_end = calculate_transformation_matrix(total_transformations, scene.translations, scene.rotations, scene.scalings, scene.composites);
            auto end_bbox = std::make_shared<bbox>(triangle_holder, 0,1,
                                                   true, 1,iterator, scene.materials[tri.material_id-1],
                                                   tri.is_light, false, tri.radiance, tri.texture_ids);

            end_bbox->apply_transformation(tr_matrix_end);
            end_bbox->transformation_matrix = tr_matrix_end;


            std::vector<parser::Transformation> blur_vec;
            blur_vec.push_back(new_mot_blur);
            Eigen::Matrix4f motion_blur_matrix_calc = calculate_transformation_matrix(blur_vec, scene.translations, scene.rotations, scene.scalings,scene.composites);

            //combine
            auto combined_bbox = std::make_shared<bbox>(motion_blur_matrix_calc,new_bbox, end_bbox,
                                                        true, 1,  iterator, scene.materials[tri.material_id-1],
                                                        tri.is_light, false,tri.radiance, tri.texture_ids);


            combined_bbox->transformation_matrix = tr_matrix;

            tlas_bboxes.push_back(combined_bbox);
        }else{
            tlas_bboxes.push_back(new_bbox);
        }

        iterator++;
        motion_blur_iterator++;
        triangle_id++;
        triangle_holder.clear();
    }


    //Mesh start********************************************

    std::vector<std::shared_ptr<mesh>> meshes;
    std::cout<<"start putting meshes"<<std::endl;
    for(auto &mes : scene.meshes){

        std::vector<parser::Vec3i> faces;
        for(auto &face : mes.faces){
            parser::Vec3i new_face;
            new_face.x = face.v0_id;
            new_face.y = face.v1_id;
            new_face.z = face.v2_id;

            faces.push_back(new_face);
        }

        std::cout<<"new mesh texture offset: "<<mes.texture_offset<<std::endl;
        auto new_mesh = std::make_shared<mesh>(mes.id, mes.material_id, mes.texture_ids,scene.materials,
                                               faces, scene.tex_coord_data, scene.vertex_data, mes.transformations, scene.translations,
                                               scene.rotations, scene.scalings, scene.composites,mes.vertex_offset, mes.texture_offset,
                                               mes.radiance, mes.is_light, mes.is_cloud);

        meshes.push_back(new_mesh);

        //blas
        std::vector<std::shared_ptr<shape>> mesh_triangles_holder;

        for(auto &triangle:new_mesh->get_triangles()){
            mesh_triangles_holder.push_back(triangle);
        }
        int count = 0;
        bvh_node* bvh = build_bvh_blas(mesh_triangles_holder, 0, mesh_triangles_holder.size(), count);

        blas_hiers.push_back(bvh);
        blas_primitives.push_back(mesh_triangles_holder);


        std::shared_ptr<bbox> new_bbox;
        Eigen::Matrix4f tr_matrix;

        //tlas
        if(new_mesh->get_transformation_matrix() == Eigen::Matrix4f::Identity()){
            new_bbox = std::make_shared<bbox>(mesh_triangles_holder, 0, mesh_triangles_holder.size(), true,
                                                   2, iterator, scene.materials[mes.material_id-1],
                                                   mes.is_light, mes.is_cloud,mes.radiance, mes.texture_ids);
            tr_matrix = Eigen::Matrix4f::Identity();
        }else{

            tr_matrix = new_mesh->get_transformation_matrix();

            new_bbox = std::make_shared<bbox>(mesh_triangles_holder, 0, mesh_triangles_holder.size(),
                                                   true, 2, iterator, scene.materials[mes.material_id-1],
                                                   mes.is_light, mes.is_cloud, mes.radiance, mes.texture_ids);
            new_bbox->apply_transformation(tr_matrix);
            new_bbox->transformation_matrix = tr_matrix;



        }

        if(mes.motion_blur.x != 0 || mes.motion_blur.y != 0 || mes.motion_blur.z != 0){
            //motion blur end bbox
            parser::Translation mot_blur;
            mot_blur.id = scene.translations.size();
            mot_blur.translation_x = mes.motion_blur.x;
            mot_blur.translation_y = mes.motion_blur.y;
            mot_blur.translation_z = mes.motion_blur.z;



            scene.translations.push_back(mot_blur);

            parser::Transformation new_mot_blur;
            new_mot_blur.transformation_type = 0;
            new_mot_blur.id = mot_blur.id +1;
            std::vector<parser::Transformation> total_transformations = mes.transformations;

            total_transformations.insert(total_transformations.begin(),new_mot_blur);

            Eigen::Matrix4f tr_matrix_end = calculate_transformation_matrix(total_transformations, scene.translations, scene.rotations, scene.scalings, scene.composites);
            auto end_bbox = std::make_shared<bbox>(mesh_triangles_holder, 0,
                                                   mesh_triangles_holder.size(),
                                                   true, 2,iterator, scene.materials[mes.material_id-1],
                                                   mes.is_light, mes.is_cloud,mes.radiance,mes.texture_ids);

            end_bbox->apply_transformation(tr_matrix_end);
            end_bbox->transformation_matrix = tr_matrix_end;


            std::vector<parser::Transformation> blur_vec;
            blur_vec.push_back(new_mot_blur);
            Eigen::Matrix4f motion_blur_matrix_calc = calculate_transformation_matrix(blur_vec, scene.translations, scene.rotations, scene.scalings, scene.composites);

            //combine
            auto combined_bbox = std::make_shared<bbox>(motion_blur_matrix_calc,new_bbox, end_bbox,
                                                        true, 2,  iterator, scene.materials[mes.material_id-1],
                                                        mes.is_light, mes.is_cloud,mes.radiance, mes.texture_ids);


            combined_bbox->transformation_matrix = tr_matrix;

            tlas_bboxes.push_back(combined_bbox);
        }else{
            tlas_bboxes.push_back(new_bbox);
        }

        mesh_triangles_holder.clear();
        iterator++;
        motion_blur_iterator++;
    }


    //Mesh Instance Start********************************************
    std::cout<<"start putting instances"<<std::endl;
    for(auto &mes_instance:scene.mesh_instances){
        std::vector<std::shared_ptr<shape>> transform_mesh_triangles_holder;
        Eigen::Matrix4f tr_matrix = Eigen::Matrix4f::Identity();
        std::vector<parser::Transformation> total_transformations = mes_instance.transformations;

        bool got_done_with_instances = false;
        int used_material_id = -1;
        int cor_mesh_id_holder = mes_instance.corresponding_mesh_id;
        while(!got_done_with_instances){
            std::cout<<"find instances"<<std::endl;
            bool got_all_instances = true;
            for(auto &mes_instance_iterator:scene.mesh_instances){

                std::cout<<cor_mesh_id_holder<<std::endl;
                std::cout<<mes_instance_iterator.id<<std::endl;
                if(cor_mesh_id_holder == mes_instance_iterator.id){
                    if(!mes_instance.reset_transform){
                        total_transformations.insert(total_transformations.begin(),
                                                     mes_instance_iterator.transformations.begin(),
                                                     mes_instance_iterator.transformations.end());
                    }
                    got_all_instances = false;
                    cor_mesh_id_holder = mes_instance_iterator.corresponding_mesh_id;
                    break;
                }

            }

            if(got_all_instances){
                got_done_with_instances = true;
            }
        }


        parser::Mesh used_mesh;
        for(auto &mes:scene.meshes){
            if(cor_mesh_id_holder == mes.id){
                used_mesh = mes;
                break;
            }
        }

        //found actual mesh
        if(!mes_instance.reset_transform){
            total_transformations.insert(total_transformations.begin(),
                                         used_mesh.transformations.begin(),
                                         used_mesh.transformations.end());
        }


        //we got all transformations upto this point
        tr_matrix = calculate_transformation_matrix(total_transformations, scene.translations, scene.rotations, scene.scalings, scene.composites);
        if(mes_instance.is_new_material){
            used_material_id = mes_instance.material_id;
        }else{
            used_material_id = used_mesh.material_id;
        }

        int mesh_in_class_id = 0;
        for(int k = 0; k < meshes.size(); k++){
            if(used_mesh.id == meshes[k]->id){
                mesh_in_class_id = k;
            }
        }


        auto new_bbox = std::make_shared<bbox>(meshes[mesh_in_class_id]->get_triangles_shape(), 0,
                                                   meshes[mesh_in_class_id]->get_triangles_shape().size(),
                                                   true, 2, mesh_in_class_id,
                                                   scene.materials[used_material_id-1],
                                                   mes_instance.is_light, false,mes_instance.radiance,mes_instance.texture_ids);

        new_bbox->apply_transformation(tr_matrix);
        new_bbox->transformation_matrix = tr_matrix;

        if(mes_instance.motion_blur.x != 0 || mes_instance.motion_blur.y != 0 || mes_instance.motion_blur.z != 0){
            //motion blur end bbox
            parser::Translation mot_blur;
            mot_blur.id = scene.translations.size();
            mot_blur.translation_x = mes_instance.motion_blur.x;
            mot_blur.translation_y = mes_instance.motion_blur.y;
            mot_blur.translation_z = mes_instance.motion_blur.z;

            scene.translations.push_back(mot_blur);

            parser::Transformation new_mot_blur;
            new_mot_blur.transformation_type = 0;
            new_mot_blur.id = mot_blur.id + 1;

            total_transformations.insert(total_transformations.begin(),new_mot_blur);

            Eigen::Matrix4f tr_matrix_end = calculate_transformation_matrix(total_transformations, scene.translations, scene.rotations, scene.scalings, scene.composites);
            auto end_bbox = std::make_shared<bbox>(meshes[mesh_in_class_id]->get_triangles_shape(), 0,
                                                   meshes[mesh_in_class_id]->get_triangles_shape().size(),
                                                   true, 2, mesh_in_class_id,
                                                   scene.materials[used_material_id-1],
                                                   mes_instance.is_light,
                                                   false,
                                                   mes_instance.radiance,
                                                   mes_instance.texture_ids);

            end_bbox->apply_transformation(tr_matrix_end);
            end_bbox->transformation_matrix = tr_matrix_end;

            std::vector<parser::Transformation> blur_vec;
            blur_vec.push_back(new_mot_blur);
            Eigen::Matrix4f motion_blur_matrix_calc = calculate_transformation_matrix(blur_vec, scene.translations, scene.rotations, scene.scalings, scene.composites);

            std::cout<<motion_blur_matrix_calc<<std::endl;
            //combine
            std::vector<std::shared_ptr<shape>> motion_blur_bboxes;
            motion_blur_bboxes.push_back(new_bbox);
            motion_blur_bboxes.push_back(end_bbox);
            auto combined_bbox = std::make_shared<bbox>(motion_blur_matrix_calc,new_bbox, end_bbox,
                                                        true, 2, mesh_in_class_id,
                                                        scene.materials[used_material_id-1],
                                                        mes_instance.is_light,
                                                        false,
                                                        mes_instance.radiance,
                                                        mes_instance.texture_ids);

            combined_bbox->transformation_matrix = tr_matrix;
            combined_bbox->motion_blur_matrix = motion_blur_matrix_calc;

            tlas_bboxes.push_back(combined_bbox);
        }else{
            tlas_bboxes.push_back(new_bbox);
        }


        motion_blur_iterator++;
    }


    //**************************************************************************************************************************************************************

    std::cout<<"meshes"<<std::endl;
    for(auto & mesh: meshes){

        triangles.insert(triangles.end(),mesh->get_triangles().begin(), mesh->get_triangles().end());
    }


    std::vector<std::shared_ptr<shape>> all_shapes;

    for(auto & cam : cameras){
        int height = cam.get_image_height();
        int width = cam.get_image_width();

        std::vector<float> color_values;

        color_values.resize(cam.get_image_height() * cam.get_image_width() * 3);

        std::vector<float> saturated_vec;
        saturated_vec.resize(cam.get_image_height() * cam.get_image_width() * 3);

        std::vector<parser::Vec3f> triangle_normals;

        for(auto &sphere:spheres){
            all_shapes.push_back(sphere);
        }

        for(auto &triangle:triangles){
            //complete backface discard
            /*
            float backface_comp_gaze = dot_product(cam.get_gaze(),triangle->get_normal(triangle->get_corners()[0]));
            float backface_comp_tl = dot_product(directions[0],triangle->get_normal(triangle->get_corners()[0]));
            float backface_comp_tr = dot_product(directions[width-1],triangle->get_normal(triangle->get_corners()[0]));
            float backface_comp_bl = dot_product(directions[width*(height-1)],triangle->get_normal(triangle->get_corners()[0]));
            float backface_comp_br = dot_product(directions[directions.size()-1],triangle->get_normal(triangle->get_corners()[0]));
            if((backface_comp_gaze < 0.0 || backface_comp_tl < 0.0 || backface_comp_tr < 0.0 || backface_comp_bl < 0.0 || backface_comp_br < 0.0) || triangle->get_type() == 2){
                all_shapes.push_back(triangle);
                parser::Vec3f hold{0,0,0};
                triangle_normals.push_back(triangle->get_normal(hold));
            }
            */
            all_shapes.push_back(triangle);
            parser::Vec3f hold{0,0,0};
            triangle_normals.push_back(triangle->get_normal(hold));
        }



        std::vector<std::shared_ptr<shape>> all_bboxes;
        parser::Vec3i object_bbox_counts{0,0,0};

        for(auto &object:tlas_bboxes){

            all_bboxes.push_back(object);

        }

        int count = 0;
        bvh_node* bvh = build_bvh_tlas(all_bboxes, 0, all_bboxes.size(), count);


        auto mid = std::chrono::high_resolution_clock::now();

        auto mid_duration = std::chrono::duration_cast<std::chrono::microseconds>(mid - start).count();

        std::cout << "Time taken to build bvh: " << mid_duration/1000000.0 << " seconds" << std::endl;
        unsigned char* image = new unsigned char [width * height * 3];

        for(int i = 0; i< (width * height * 3)-1;i++){
            image[i] = 0.0;
        }

        uint32_t processing_unit_count = std::thread::hardware_concurrency();
        if (processing_unit_count == 0) {
            processing_unit_count = 8;
        }

        std::vector<std::thread> processing_units;
        processing_units.reserve(processing_unit_count);

        std::atomic<uint32_t> cursor = 0;


        //Line-by-line structure
        /*
        for (uint32_t i = 0; i < processing_unit_count; i++) {
            processing_units.push_back(std::thread([&]() {
                while (true) {
                    uint32_t j = cursor.fetch_add(1, std::memory_order_relaxed);
                    if (j >= height) {
                        break;
                    }
                    //std::cout<<j<<std::endl;
                    parser::Vec3f current_direction;
                    parser::Vec3f color;
                    for (uint32_t i = 0; i < width; i++) {
                        int depth = 0;
                        current_direction = directions[(j*width)+i];

                        parser::Vec3f background_color_3f;
                        background_color_3f.x = scene.background_color.x;
                        background_color_3f.y = scene.background_color.y;
                        background_color_3f.z = scene.background_color.z;

                        color = compute_color(cam.get_position(), current_direction,
                                              point_lights,all_shapes,background_color_3f,
                                              depth, scene.max_recursion_depth, scene.ambient_light,
                                              scene.shadow_ray_epsilon, triangle_normals, bvh, 0,
                                              blas_hiers, object_bbox_counts,blas_primitives, spheres, triangles, meshes);

                        image[((j*width)+i)*3] = std::round(clamp(color.x));
                        image[(((j*width)+i)*3)+1] = std::round(clamp(color.y));
                        image[(((j*width)+i)*3)+2] = std::round(clamp(color.z));

                    }

                }
            }));
        }

        for (auto &processing_unit: processing_units) {
            processing_unit.join();
        }
        */



        //new way for multisampling**********************************************************************************************************************************************************************************************

        //holds an array of all pixels in order

        if (cam.get_num_samples() > 1) {

            std::vector<std::vector<parser::Vec3f>> all_pixel_colors(height * width, std::vector<parser::Vec3f>(cam.get_num_samples()));

            std::vector<ray> rays;
            rays.resize(cam.get_image_width()*cam.get_image_height()*cam.get_num_samples());

            if (cam.get_aperture_size() > 0.0) {
                cam.get_jittered_ray_directions_dof(cam.get_num_samples(), rays);
            } else {
                cam.get_jittered_ray_directions(cam.get_num_samples(), rays);
            }
            auto ray_stop = std::chrono::high_resolution_clock::now();

            auto cur_dur = std::chrono::duration_cast<std::chrono::microseconds>(ray_stop - start).count();

            std::cout << "Time taken until ray creation ends: " << cur_dur/1000000.0 << " seconds" << std::endl;

            for (uint32_t i = 0; i < processing_unit_count; i++) {
                processing_units.push_back(std::thread([&]() {
                    std::random_device rd;
                    static thread_local std::mt19937 random_generator(rd() + i);
                    std::cout<<"bupbap"<<std::endl;
                    while (true) {
                        uint32_t j = cursor.fetch_add(1, std::memory_order_relaxed);
                        if (j >= height) {
                            break;
                        }

                        parser::Vec3f current_direction;
                        parser::Vec3f current_pos;
                        parser::Vec3f color;
                        float time;
                        std::cout<<j<<std::endl;
                        for (uint32_t i = 0; i < width; i++) {
                            std::vector<parser::Vec3f> colors_in_pixel(cam.get_num_samples());
                            //area sampling

                            std::vector<std::vector<std::pair<float,float>>> all_area_samples;

                            for(auto light:area_lights){
                                std::vector<std::pair<float,float>> one_light_area_samples = generate_samples_jittered(cam.get_num_samples(),
                                                                                                                       pow(light.get_size(),2),false, random_generator);

                                //std::shuffle(one_light_area_samples.begin(), one_light_area_samples.end(),gRandomGenerator);
                                all_area_samples.push_back(one_light_area_samples);
                            }

                            for (int k = 0; k < cam.get_num_samples(); k++) {
                                size_t ray_index = (j * width + i) * cam.get_num_samples() + k;
                                current_direction = rays[ray_index].direction;
                                current_pos = rays[ray_index].start_pos;
                                time = rays[ray_index].time;

                                parser::Vec3f background_color_3f{};
                                bool background_tex = false;

                                //add background texture
                                for(auto &tex: textures){
                                    if(tex.decal_mode == 3){
                                        background_tex = true;
                                        parser::Vec2f uv_background;
                                        uv_background.u = rays[ray_index].near_plane_uv.first;
                                        uv_background.v = rays[ray_index].near_plane_uv.second;
                                        background_color_3f = tex.get_texture_value(uv_background);

                                    }
                                }

                                if(!background_tex){
                                    background_color_3f.x = scene.background_color.x;
                                    background_color_3f.y = scene.background_color.y;
                                    background_color_3f.z = scene.background_color.z;
                                }




                                parser::Vec3f gaze_hold = cam.get_gaze();

                                std::vector<std::pair<float,float>> one_ray_area_samples;
                                one_ray_area_samples.reserve(area_lights.size());
                                for(int t = 0; t < area_lights.size(); t++){
                                    one_ray_area_samples.push_back(all_area_samples[t][k]);
                                }


                                int depth = 0;

                                if(!cam.is_path_tracing()){
                                    color = compute_color(
                                            textures,
                                            random_generator,
                                            one_ray_area_samples,
                                            current_pos,
                                            current_direction,
                                            time,
                                            cam.get_near_distance(),
                                            true,
                                            cam.get_ignore(),
                                            gaze_hold,
                                            point_lights,
                                            area_lights,
                                            directional_lights,
                                            spot_lights,
                                            environment_lights,
                                            all_shapes,
                                            background_color_3f,
                                            depth,
                                            scene.max_recursion_depth,
                                            scene.ambient_light,
                                            scene.shadow_ray_epsilon,
                                            triangle_normals,
                                            bvh,
                                            0,
                                            blas_hiers,
                                            object_bbox_counts,
                                            blas_primitives,
                                            spheres,
                                            triangles,
                                            meshes,
                                            brdfs,
                                            scene.cloud_parameters
                                    );
                                }else{
                                    float throughput = 1.0f;
                                    int path_depth = 0;
                                    color = path_trace(
                                            textures,
                                            random_generator,
                                            one_ray_area_samples,
                                            current_pos,
                                            current_direction,
                                            time,
                                            cam.get_near_distance(),
                                            true,
                                            cam.get_ignore(),
                                            gaze_hold,
                                            point_lights,
                                            area_lights,
                                            directional_lights,
                                            spot_lights,
                                            environment_lights,
                                            all_shapes,
                                            background_color_3f,
                                            depth,
                                            scene.max_recursion_depth,
                                            scene.min_recursion_depth,
                                            scene.shadow_ray_epsilon,
                                            triangle_normals,
                                            bvh,
                                            0,
                                            blas_hiers,
                                            object_bbox_counts,
                                            blas_primitives,
                                            spheres,
                                            triangles,
                                            meshes,
                                            brdfs,
                                            cam,
                                            throughput,
                                            path_depth
                                    );
                                }


                                colors_in_pixel[k] = color;
                            }

                            all_pixel_colors[j * width + i] = colors_in_pixel;
                        }
                    }
                }));
            }


            for (auto &processing_unit : processing_units) {
                processing_unit.join();
            }
            auto shading_stop = std::chrono::high_resolution_clock::now();

            auto shading_dur = std::chrono::duration_cast<std::chrono::microseconds>(shading_stop - start).count();

            std::cout << "Time taken until shading ends: " << shading_dur/1000000.0 << " seconds" << std::endl;

            processing_units.clear();


            processing_units.reserve(processing_unit_count);
            cursor = 0;


            for (uint32_t z = 0; z < processing_unit_count; z++) {
                processing_units.push_back(std::thread([&]() {
                    while (true) {
                        uint32_t h = cursor.fetch_add(1, std::memory_order_relaxed);
                        if (h >= height) {
                            break;
                        }

                        float norm = 6.0;
                        std::cout<<h<<std::endl;
                        for (int i = 0; i < width; i++) {
                            float gaussian_sum = 0.0f;
                            int total_gaussian_count = 0;
                            std::vector<parser::Vec3f> collected_colors;
                            std::vector<float> collected_gaussian;
                            float u_dev = 0.0;
                            float v_dev = 0.0;
                            for (int j = 0; j < cam.get_num_samples(); j++) {
                                size_t ray_index = (h * width + i) * cam.get_num_samples() + j;
                                u_dev = rays[ray_index].pixel_cast_u;
                                v_dev = rays[ray_index].pixel_cast_v;

                                float u_diff = fabs((u_dev - 0.5f) * cam.pixel_size);
                                float v_diff = fabs((v_dev - 0.5f) * cam.pixel_size);

                                float gaussian = get_gaussian_value(u_diff, v_diff, cam.pixel_size / norm);

                                gaussian_sum += gaussian;
                                collected_gaussian.push_back(gaussian);
                                collected_colors.push_back(all_pixel_colors[h * width + i][j]);
                                total_gaussian_count++;
                            }


                            //directs********************************************
                            //top

                            if(h > 0){
                                for(int j = 0; j < cam.get_num_samples(); j++){

                                    u_dev = rays[((h-1)*width + i)*cam.get_num_samples() + j].pixel_cast_u;
                                    v_dev = rays[((h-1)*width + i)*cam.get_num_samples() + j].pixel_cast_v;

                                    //mid_point u is 0.5 from top left
                                    float u_diff = fabs((u_dev - 0.5)*cam.pixel_size);

                                    //mid_point v is -0.5 from top left
                                    float v_diff = fabs((1.5 -v_dev)*cam.pixel_size);

                                    float gaussian = get_gaussian_value(u_diff, v_diff, cam.pixel_size/ norm);

                                    gaussian_sum += gaussian;
                                    collected_gaussian.push_back(gaussian);
                                    collected_colors.push_back(all_pixel_colors[(h-1)*width + i][j]);
                                    total_gaussian_count++;

                                }
                            }


                            //bottom

                            if(h < (height-1)){
                                for(int j = 0; j < cam.get_num_samples(); j++){

                                    u_dev = rays[((h+1)*width + i)*cam.get_num_samples() + j].pixel_cast_u;
                                    v_dev = rays[((h+1)*width + i)*cam.get_num_samples() + j].pixel_cast_v;

                                    //mid_point u is 0.5 from top left
                                    float u_diff = fabs((u_dev - 0.5)*cam.pixel_size);

                                    //mid_point v is -0.5 from top left
                                    float v_diff = fabs((1.0 + v_dev - 0.5)*cam.pixel_size);

                                    float gaussian = get_gaussian_value(u_diff, v_diff, cam.pixel_size/ norm);

                                    gaussian_sum += gaussian;
                                    collected_gaussian.push_back(gaussian);
                                    collected_colors.push_back(all_pixel_colors[(h+1)*width + i][j]);
                                    total_gaussian_count++;

                                }
                            }


                            //left
                            if(i > 0){
                                for(int j = 0; j < cam.get_num_samples(); j++){

                                    u_dev = rays[(h*width + i - 1)*cam.get_num_samples() + j].pixel_cast_u;
                                    v_dev = rays[(h*width + i - 1)*cam.get_num_samples() + j].pixel_cast_v;

                                    //mid_point u is 0.5 from top left
                                    float u_diff = fabs((1.5 - u_dev)*cam.pixel_size);

                                    //mid_point v is -0.5 from top left
                                    float v_diff = fabs((v_dev - 0.5)*cam.pixel_size);

                                    float gaussian = get_gaussian_value(u_diff, v_diff, cam.pixel_size/ norm);

                                    gaussian_sum += gaussian;
                                    collected_gaussian.push_back(gaussian);
                                    collected_colors.push_back(all_pixel_colors[h*width + (i - 1)][j]);
                                    total_gaussian_count++;

                                }
                            }


                            //right
                            if(i < (width-1)){
                                for(int j = 0; j < cam.get_num_samples(); j++){

                                    u_dev = rays[(h*width + i + 1)*cam.get_num_samples() + j].pixel_cast_u;
                                    v_dev = rays[(h*width + i + 1)*cam.get_num_samples() + j].pixel_cast_v;

                                    //mid_point u is 0.5 from top left
                                    float u_diff = fabs((u_dev + 1.0 - 0.5)*cam.pixel_size);

                                    //mid_point v is -0.5 from top left
                                    float v_diff = fabs((1.0+v_dev - 0.5)*cam.pixel_size);

                                    float gaussian = get_gaussian_value(u_diff, v_diff, cam.pixel_size/ norm);

                                    gaussian_sum += gaussian;
                                    collected_gaussian.push_back(gaussian);
                                    collected_colors.push_back(all_pixel_colors[h*width + (i + 1)][j]);
                                    total_gaussian_count++;

                                }
                            }


                            //diagonals********************************************

                            //top left

                            if(i > 0 && h > 0){
                                for(int j = 0; j < cam.get_num_samples(); j++){

                                    u_dev = rays[((h-1)*width + i - 1)*cam.get_num_samples() + j].pixel_cast_u;
                                    v_dev = rays[((h-1)*width + i - 1)*cam.get_num_samples() + j].pixel_cast_v;

                                    //mid_point u is 0.5 from top left
                                    float u_diff = fabs((1.5 - u_dev)*cam.pixel_size);

                                    //mid_point v is -0.5 from top left
                                    float v_diff = fabs((v_dev - 1.5)*cam.pixel_size);

                                    float gaussian = get_gaussian_value(u_diff, v_diff, cam.pixel_size/ norm);

                                    gaussian_sum += gaussian;
                                    collected_gaussian.push_back(gaussian);
                                    collected_colors.push_back(all_pixel_colors[(h-1)*width + (i - 1)][j]);
                                    total_gaussian_count++;

                                }
                            }


                            //top right
                            if( i < (width-1) && h > 0){
                                for(int j = 0; j < cam.get_num_samples(); j++){

                                    u_dev = rays[((h-1)*width + i + 1)*cam.get_num_samples() + j].pixel_cast_u;
                                    v_dev = rays[((h-1)*width + i + 1)*cam.get_num_samples() + j].pixel_cast_v;

                                    //mid_point u is 0.5 from top left
                                    float u_diff = fabs((u_dev + 1.0 - 0.5)*cam.pixel_size);

                                    //mid_point v is -0.5 from top left
                                    float v_diff = fabs((v_dev - 1.5)*cam.pixel_size);

                                    float gaussian = get_gaussian_value(u_diff, v_diff, cam.pixel_size/ norm);

                                    gaussian_sum += gaussian;
                                    collected_gaussian.push_back(gaussian);
                                    collected_colors.push_back(all_pixel_colors[(h-1)*width + (i + 1)][j]);
                                    total_gaussian_count++;

                                }
                            }


                            //bottom left
                            if(i > 0 && h < (height-1)){
                                for(int j = 0; j < cam.get_num_samples(); j++){

                                    u_dev = rays[((h+1)*width + i - 1)*cam.get_num_samples() + j].pixel_cast_u;
                                    v_dev = rays[((h+1)*width + i - 1)*cam.get_num_samples() + j].pixel_cast_v;

                                    //mid_point u is 0.5 from top left
                                    float u_diff = fabs((1.5 - u_dev)*cam.pixel_size);

                                    //mid_point v is -0.5 from top left
                                    float v_diff = fabs((1.0 + v_dev - 0.5)*cam.pixel_size);

                                    float gaussian = get_gaussian_value(u_diff, v_diff, cam.pixel_size/ norm);

                                    gaussian_sum += gaussian;
                                    collected_gaussian.push_back(gaussian);
                                    collected_colors.push_back(all_pixel_colors[(h+1)*width + (i - 1)][j]);
                                    total_gaussian_count++;

                                }
                            }


                            //bottom right
                            if(i < (width-1) && h < (height-1)){
                                for(int j = 0; j < cam.get_num_samples(); j++){

                                    u_dev = rays[((h+1)*width + i + 1)*cam.get_num_samples() + j].pixel_cast_u;
                                    v_dev = rays[((h+1)*width + i + 1)*cam.get_num_samples() + j].pixel_cast_v;

                                    //mid_point u is 0.5 from top left
                                    float u_diff = fabs((u_dev + 1.0 - 0.5)*cam.pixel_size);

                                    //mid_point v is -0.5 from top left
                                    float v_diff = fabs((1.0 + v_dev - 0.5)*cam.pixel_size);

                                    float gaussian = get_gaussian_value(u_diff, v_diff, cam.pixel_size/ norm);

                                    gaussian_sum += gaussian;
                                    collected_gaussian.push_back(gaussian);
                                    collected_colors.push_back(all_pixel_colors[(h+1)*width + (i + 1)][j]);
                                    total_gaussian_count++;

                                }
                            }

                            parser::Vec3f total_color{0.0f, 0.0f, 0.0f};

                            for (int j = 0; j < total_gaussian_count; j++) {
                                if (gaussian_sum > 0) {
                                    total_color.x += (collected_gaussian[j] * collected_colors[j].x) / gaussian_sum;
                                    total_color.y += (collected_gaussian[j] * collected_colors[j].y) / gaussian_sum;
                                    total_color.z += (collected_gaussian[j] * collected_colors[j].z) / gaussian_sum;
                                }
                            }

                            color_values[(h * width + i) * 3 + 0] = total_color.x;
                            color_values[(h * width + i) * 3 + 1] = total_color.y;
                            color_values[(h * width + i) * 3 + 2] = total_color.z;

                        }
                    }
                }));
            }

            // Wait for all threads to finish
            for (auto &processing_unit : processing_units) {
                processing_unit.join();
            }



        }else{
            std::vector<ray> default_rays = cam.get_ray_directions();


            for (uint32_t i = 0; i < processing_unit_count; i++) {
                processing_units.push_back(std::thread([&]() {
                    std::random_device rd;
                    static thread_local std::mt19937 random_generator(rd() + i);
                    while (true) {
                        uint32_t j = cursor.fetch_add(1, std::memory_order_relaxed);
                        if (j >= height) {
                            break;
                        }


                        std::cout<<j<<std::endl;
                        parser::Vec3f current_direction;
                        parser::Vec3f color;
                        for (uint32_t i = 0; i < width; i++) {
                            int depth = 0;
                            current_direction = default_rays[(j*width)+i].direction;
                            float time = 0;

                            std::vector<std::pair<float,float>> empty_area_samples;

                            parser::Vec3f background_color_3f{};
                            bool background_tex = false;

                            //add background texture
                            for(auto &tex: textures){
                                if(tex.decal_mode == 3){

                                    background_tex = true;
                                    parser::Vec2f uv_background;
                                    uv_background.u = default_rays[(j*width)+i].near_plane_uv.first;
                                    uv_background.v = default_rays[(j*width)+i].near_plane_uv.second;
                                    background_color_3f = tex.get_texture_value(uv_background);

                                }
                            }

                            if(!background_tex){
                                background_color_3f.x = scene.background_color.x;
                                background_color_3f.y = scene.background_color.y;
                                background_color_3f.z = scene.background_color.z;
                            }

                            parser::Vec3f gaze_hold = cam.get_gaze();

                            if(!cam.is_path_tracing()){
                                color = compute_color(
                                        textures,
                                        random_generator,
                                        empty_area_samples,
                                        cam.get_position(),
                                        current_direction,
                                        time,
                                        cam.get_near_distance(),
                                        true,
                                        cam.get_ignore(),
                                        gaze_hold,
                                        point_lights,
                                        area_lights,
                                        directional_lights,
                                        spot_lights,
                                        environment_lights,
                                        all_shapes,
                                        background_color_3f,
                                        depth,
                                        scene.max_recursion_depth,
                                        scene.ambient_light,
                                        scene.shadow_ray_epsilon,
                                        triangle_normals,
                                        bvh,
                                        0,
                                        blas_hiers,
                                        object_bbox_counts,
                                        blas_primitives,
                                        spheres,
                                        triangles,
                                        meshes,
                                        brdfs,
                                        scene.cloud_parameters
                                );
                            }else{
                                float throughput = 1.0f;
                                int path_depth = 0;
                                color = path_trace(
                                        textures,
                                        random_generator,
                                        empty_area_samples,
                                        cam.get_position(),
                                        current_direction,
                                        time,
                                        cam.get_near_distance(),
                                        true,
                                        cam.get_ignore(),
                                        gaze_hold,
                                        point_lights,
                                        area_lights,
                                        directional_lights,
                                        spot_lights,
                                        environment_lights,
                                        all_shapes,
                                        background_color_3f,
                                        depth,
                                        scene.max_recursion_depth,
                                        scene.min_recursion_depth,
                                        scene.shadow_ray_epsilon,
                                        triangle_normals,
                                        bvh,
                                        0,
                                        blas_hiers,
                                        object_bbox_counts,
                                        blas_primitives,
                                        spheres,
                                        triangles,
                                        meshes,
                                        brdfs,
                                        cam,
                                        throughput,
                                        path_depth
                                );
                            }



                            color_values[(j * width + i) * 3 + 0] = color.x;
                            color_values[(j * width + i) * 3 + 1] = color.y;
                            color_values[(j * width + i) * 3 + 2] = color.z;
                            /*
                            image[((j*width)+i)*3] = std::round(clamp(color.x));
                            image[(((j*width)+i)*3)+1] = std::round(clamp(color.y));
                            image[(((j*width)+i)*3)+2] = std::round(clamp(color.z));
                            */
                        }

                    }
                }));
            }

            for (auto &processing_unit: processing_units) {
                processing_unit.join();
            }


        }


        if(cam.tone_map.tmo != -1){
            std::string output_string_exr = "../my_outputs/" ;
            std::string cam_string_exr = cam.get_name();
            std::string all_string_exr = output_string_exr + cam_string_exr;

            save_exr_image(color_values, cam.get_image_width(), cam.get_image_height(),
                           all_string_exr.c_str());
        }

        if(cam.tone_map.tmo != -1){
            std::vector<float> luminance_values;
            int color_count = color_values.size();
            for(int l = 0; l < color_count; l+=3){

                parser::Vec3f rgb_value{color_values[l],
                                        color_values[l + 1],
                                        color_values[l + 2]};
                luminance_values.push_back(get_luminance(rgb_value));

            }

            float average_luminance = get_average_luminance(luminance_values, color_count/3);

            if(cam.tone_map.burn_percent == 0){
                for(int l = 0; l < luminance_values.size(); l++){
                    float yo = tone_map_zero_burnout(get_scaled_luminance(luminance_values[l],
                                                                          average_luminance,
                                                                          cam.tone_map.key_value));

                    parser::Vec3f current_color{color_values[l * 3],
                                                color_values[(l * 3) + 1],
                                                color_values[(l * 3) + 2]};

                    parser::Vec3f saturated = saturation(current_color, luminance_values[l], yo, cam.tone_map.saturation);

                    parser::Vec3f clamped{std::clamp(saturated.x, 0.0f, 1.0f),
                                          std::clamp(saturated.y, 0.0f, 1.0f),
                                          std::clamp(saturated.z, 0.0f, 1.0f)};


                    parser::Vec3f final_color = gamma_correction(clamped, cam.tone_map.gamma);

                    color_values[l*3] = final_color.x;
                    color_values[(l*3) + 1] = final_color.y;
                    color_values[(l*3) + 2] = final_color.z;

                }

            }else{
                std::vector<float> scaled_luminance_values;
                for(auto &lum: luminance_values){
                    scaled_luminance_values.push_back(get_scaled_luminance(lum,average_luminance, cam.tone_map.key_value));
                }

                float l_white = get_l_white(scaled_luminance_values, cam.tone_map.burn_percent);

                for(int l = 0; l < luminance_values.size(); l++){
                    float yo = tone_map(scaled_luminance_values[l], l_white);

                    parser::Vec3f current_color{color_values[l * 3],
                                                color_values[(l * 3) + 1],
                                                color_values[(l * 3) + 2]};

                    parser::Vec3f saturated = saturation(current_color, luminance_values[l], yo, cam.tone_map.saturation);

                    parser::Vec3f clamped{std::clamp(saturated.x, 0.0f, 1.0f),
                                          std::clamp(saturated.y, 0.0f, 1.0f),
                                          std::clamp(saturated.z, 0.0f, 1.0f)};

                    parser::Vec3f final_color = gamma_correction(clamped, cam.tone_map.gamma);


                    color_values[l*3] = final_color.x;
                    color_values[(l*3) + 1] = final_color.y;
                    color_values[(l*3) + 2] = final_color.z;

                }
            }
        }



        for(int l = 0; l < color_values.size();l++){
            image[l] = std::round(clamp(color_values[l]));
        }



        //*********************************************************************************************************************************************************************************************************************

        //average aa version
        /*
        std::vector<ray> rays = cam.get_jittered_ray_directions(cam.get_num_samples());
        std::cout<<"ray number: "<<rays.size()<<std::endl;

        //new way for multisampling
        for (uint32_t i = 0; i < processing_unit_count; i++) {
            processing_units.push_back(std::thread([&]() {
                while (true) {
                    uint32_t j = cursor.fetch_add(1, std::memory_order_relaxed);
                    if (j >= height) {
                        break;
                    }

                    std::cout<<j<<std::endl;
                    parser::Vec3f current_direction;
                    parser::Vec3f color;
                    for (uint32_t i = 0; i < width; i++) {

                        std::vector<parser::Vec3f> colors_in_pixel;

                        for(int k = 0; k < cam.get_num_samples(); k++){
                            current_direction = rays[(j * width * cam.get_num_samples()) + (i*cam.get_num_samples()) + k].direction;

                            parser::Vec3f background_color_3f;
                            background_color_3f.x = scene.background_color.x;
                            background_color_3f.y = scene.background_color.y;
                            background_color_3f.z = scene.background_color.z;


                            int depth = 0;
                            color = compute_color(cam.get_position(), current_direction,
                                                  point_lights,all_shapes,background_color_3f,
                                                  depth, scene.max_recursion_depth, scene.ambient_light,
                                                  scene.shadow_ray_epsilon, triangle_normals, bvh, 0,
                                                  blas_hiers, object_bbox_counts,blas_primitives, spheres, triangles, meshes);

                            //std::cout<<color.x<<""<<color.y<<""<<color.z<<std::endl;
                            colors_in_pixel.push_back(color);

                        }

                        parser::Vec3f color_sum{0,0,0};
                        for(int k = 0; k < colors_in_pixel.size(); k++){
                            color_sum.x += colors_in_pixel[k].x;
                            color_sum.y += colors_in_pixel[k].y;
                            color_sum.z += colors_in_pixel[k].z;
                        }

                        color_sum.x = color_sum.x/colors_in_pixel.size();
                        color_sum.y = color_sum.y/colors_in_pixel.size();
                        color_sum.z = color_sum.z/colors_in_pixel.size();


                        image[((j*width)+i)*3] = std::round(clamp(color_sum.x));
                        image[(((j*width)+i)*3)+1] = std::round(clamp(color_sum.y));
                        image[(((j*width)+i)*3)+2] = std::round(clamp(color_sum.z));

                    }

                }
            }));
        }

        for (auto &processing_unit: processing_units) {
            processing_unit.join();
        }
         */
        /*
        int counter = 0;
        parser::Vec3f current_direction;
        parser::Vec3f color;
        for(uint32_t w = 0; w < width; w++){
            std::cout<<w<<std::endl;
            for(uint32_t h = 0; h < height; h++){
                int depth = 0;
                current_direction = directions[(h*width)+w];
                parser::Vec3f background_color_3f;
                background_color_3f.x = scene.background_color.x;
                background_color_3f.y = scene.background_color.y;
                background_color_3f.z = scene.background_color.z;
                color = compute_color(cam.get_position(), current_direction,
                                      point_lights,all_shapes,background_color_3f,
                                      depth, scene.max_recursion_depth, scene.ambient_light,
                                      scene.shadow_ray_epsilon, triangle_normals, bvh, 0,
                                      blas_hiers, object_bbox_counts,blas_primitives, spheres, triangles, meshes);

                image[((h*width)+w)*3] = std::round(clamp(color.x));
                image[(((h*width)+w)*3)+1] = std::round(clamp(color.y));
                image[(((h*width)+w)*3)+2] = std::round(clamp(color.z));
            }

            counter++;
        }*/


        std::string output_string = "../my_outputs/" ;
        std::string cam_string = cam.get_name();
        std::string all_string = output_string + cam_string;

        if(all_string.substr(all_string.length() - 4) == ".exr"){
            all_string = all_string.substr(0, all_string.length() - 4) + ".png";
        }

        //write_ppm(all_string.c_str(), image, width, height);
        stbi_write_png(all_string.c_str(), width, height, 3, image, (width*3));
        auto pend = std::chrono::high_resolution_clock::now();

        auto pduration = std::chrono::duration_cast<std::chrono::microseconds>(pend - start).count();

        std::cout << "Time taken for an image: " << pduration/1000000.0 << " seconds" << std::endl;

        all_shapes.clear();
        delete bvh;

    }

    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    std::cout << "Time taken: " << duration/1000000.0 << " seconds" << std::endl;


}

