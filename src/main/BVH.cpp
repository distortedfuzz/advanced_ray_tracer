#include "BVH.h"
#include "../math/math.h"
#include <iostream>
#include <stack>
#include <utility>

bvh_node::bvh_node(bbox box, bvh_node* left_child, bvh_node* right_child)
        : bounding_box(std::move(box)), left(left_child), right(right_child), is_leaf(false) {}

bvh_node::bvh_node(bbox box, int primitive)
        : bounding_box(std::move(box)), left(nullptr), right(nullptr), is_leaf(true), primitive_id(primitive) {}

bvh_node::bvh_node(bbox box, int primitive, int blas)
        : bounding_box(std::move(box)), left(nullptr), right(nullptr), is_leaf(true), primitive_id(primitive), blas_id(blas) {}

bvh_node_linear::bvh_node_linear(bbox box, int left_child, int right_child)
        : bounding_box(std::move(box)), left_index(left_child), right_index(right_child), is_leaf(false) {}

bvh_node_linear::bvh_node_linear(bbox box, int primitive)
        : bounding_box(std::move(box)), left_index(-1), right_index(-1), is_leaf(true), primitive_id(primitive) {}


int partition(std::vector<std::shared_ptr<shape>> &shapes, int start, int end, int partition_axis, float cut_pos, int &swap_count) {

    int swap_pos = start;
    int running_idx = start;

    while(running_idx < end){
        if(partition_axis == 0){
            if(shapes[running_idx]->get_center().x < cut_pos ){
                swap(shapes[swap_pos], shapes[running_idx]);
                swap_count++;
                swap_pos++;
                running_idx++;
            }else{
                running_idx++;
            }
        }else if(partition_axis == 1){
            if(shapes[running_idx]->get_center().y < cut_pos ){
                swap(shapes[swap_pos], shapes[running_idx]);
                swap_count++;
                swap_pos++;
                running_idx++;
            }else{
                running_idx++;
            }
        }else if(partition_axis == 2){
            if(shapes[running_idx]->get_center().z < cut_pos ){
                swap(shapes[swap_pos], shapes[running_idx]);
                swap_count++;
                swap_pos++;
                running_idx++;
            }else{
                running_idx++;
            }
        }
    }

    return swap_pos;

}

bvh_node* build_bvh(std::vector<std::shared_ptr<shape>> &shapes, int start, int end, int &count) {

    bbox bounding_box(shapes, start, end);
    count++;
    if (end - start <= 1) {
        bounding_box.object_type = 0;

        return new bvh_node(bounding_box, start, 0);
    }
    parser::Vec3f dimension_differences = bounding_box.get_dimension_differences();

    float biggest_dimension_difference = fmax(dimension_differences.x,
                                              fmax(dimension_differences.y, dimension_differences.z));

    int dimension;
    float cut_pos;
    std::vector<parser::Vec3f> corners = bounding_box.get_corners();
    if(fabs(biggest_dimension_difference - dimension_differences.x) < 0.0001){
        dimension = 0;
        cut_pos = (corners[0].x + corners[1].x) / 2.0;
    }else if(fabs(biggest_dimension_difference - dimension_differences.y) < 0.0001){
        dimension = 1;
        cut_pos = (corners[0].y + corners[1].y) / 2.0;
    }else if(fabs(biggest_dimension_difference - dimension_differences.z) < 0.0001){
        dimension = 2;
        cut_pos = (corners[0].z + corners[1].z) / 2.0;
    }
    cut_pos += 0.0001;
    int swap_count = 0;
    int new_index = partition(shapes, start, end, dimension, cut_pos, swap_count);

    if (swap_count == 0 || new_index == start || new_index == end) {
        new_index = start + (end - start) / 2;
    }
    return new bvh_node(
            bounding_box,
            build_bvh(shapes, start, new_index, count),
            build_bvh(shapes, new_index, end, count)
    );
}


bool bvh_intersect(bvh_node* node, parser::Vec3f &camera_position, parser::Vec3f &ray_direction,
                   hit_record &rec, std::vector<std::shared_ptr<shape>> &shapes){

    if(!node->bounding_box.does_intersect(camera_position, ray_direction)){
        return false;
    }

    if (node->is_leaf) {
        bool intersect = false;

        if (node->primitive_id != -1) {

            rec.primitive_id = node->primitive_id;
            rec.t_variable = shapes[rec.primitive_id]->get_intersection_parameter(camera_position, ray_direction);
            rec.intersected_point = vector_add(camera_position, vector_multiply(ray_direction, rec.t_variable));
            rec.normal = shapes[rec.primitive_id]->get_normal(rec.intersected_point);
            rec.material = shapes[rec.primitive_id]->get_material_struct();
            if(rec.t_variable > 0.0){
                intersect = true;
            }
        }


        return intersect;
    }
    hit_record rec1, rec2;

    rec.t_variable = INFINITY;

    bool hit_left = bvh_intersect(node->left, camera_position, ray_direction, rec1, shapes);
    bool hit_right = bvh_intersect(node->right, camera_position, ray_direction, rec2, shapes);

    if(hit_left){
        rec = rec1;
    }

    if(hit_right){
        rec = rec2.t_variable < rec.t_variable ? rec2 : rec;
    }

    return (hit_left || hit_right);
}

bvh_node* build_bvh_blas(std::vector<std::shared_ptr<shape>> &shapes, int start, int end, int &count) {

    bbox bounding_box(shapes, start, end);
    count++;
    if (end - start <= 1) {
        bounding_box.object_type = -1;
        return new bvh_node(bounding_box, -1, start);
    }
    parser::Vec3f dimension_differences = bounding_box.get_dimension_differences();

    float biggest_dimension_difference = fmax(dimension_differences.x,
                                              fmax(dimension_differences.y, dimension_differences.z));

    int dimension;
    float cut_pos;
    std::vector<parser::Vec3f> corners = bounding_box.get_corners();
    if(fabs(biggest_dimension_difference - dimension_differences.x) < 0.00001){
        dimension = 0;
        cut_pos = (corners[0].x + corners[1].x) / 2.0;
    }else if(fabs(biggest_dimension_difference - dimension_differences.y) < 0.00001){
        dimension = 1;
        cut_pos = (corners[0].y + corners[1].y) / 2.0;
    }else if(fabs(biggest_dimension_difference - dimension_differences.z) < 0.00001){
        dimension = 2;
        cut_pos = (corners[0].z + corners[1].z) / 2.0;
    }

    int swap_count = 0;
    int new_index = partition(shapes, start, end, dimension, cut_pos, swap_count);

    if (swap_count == 0 || new_index == start || new_index == end) {
        new_index = start + (end - start) / 2;
    }
    return new bvh_node(
            bounding_box,
            build_bvh_blas(shapes, start, new_index, count),
            build_bvh_blas(shapes, new_index, end, count)
    );
}



bvh_node* build_bvh_tlas(std::vector<std::shared_ptr<shape>> &shapes, int start, int end, int &count) {

    bbox bounding_box(shapes, start, end);

    count++;
    if (end - start <= 1) {
        if(shapes[start]->get_shape_inside_shape_type() != -1){
            bounding_box.object_type = shapes[start]->get_shape_inside_shape_type();
            bounding_box.motion_blur_matrix = shapes[start]->get_blur_matrix();
            bounding_box.is_cloud = shapes[start]->get_is_cloud();
            std::cout<<bounding_box.motion_blur_matrix<<std::endl;
            bounding_box.object_index_in_tlas_vector = shapes[start]->get_index_in_tlas();
            bounding_box.transformation_matrix = shapes[start]->get_transformation_matrix();
            bounding_box.bbox_material = shapes[start]->get_material_struct();
            bounding_box.texture_ids = shapes[start]->get_tex_ids();
            bounding_box.is_light = shapes[start]->get_is_light();
            std::cout<<"is light bvh "<<bounding_box.is_light<<std::endl;

            if(bounding_box.is_light){
                std::cout<<"yessir"<<std::endl;
                bounding_box.radiance = shapes[start]->get_radiance();
            }else{
                std::cout<<"no sir"<<std::endl;
                bounding_box.radiance.x = 0.0;
                bounding_box.radiance.y = 0.0;
                bounding_box.radiance.z = 0.0;
            }


        }
        return new bvh_node(bounding_box, start);
    }
    parser::Vec3f dimension_differences = bounding_box.get_dimension_differences();

    float biggest_dimension_difference = fmax(dimension_differences.x,
                                              fmax(dimension_differences.y, dimension_differences.z));

    int dimension;
    float cut_pos;
    std::vector<parser::Vec3f> corners = bounding_box.get_corners();
    if(fabs(biggest_dimension_difference - dimension_differences.x) < 0.000001){
        dimension = 0;
        cut_pos = (corners[0].x + corners[1].x) / 2.0;
    }else if(fabs(biggest_dimension_difference - dimension_differences.y) < 0.000001){
        dimension = 1;
        cut_pos = (corners[0].y + corners[1].y) / 2.0;
    }else if(fabs(biggest_dimension_difference - dimension_differences.z) < 0.000001){
        dimension = 2;
        cut_pos = (corners[0].z + corners[1].z) / 2.0;
    }

    int swap_count = 0;
    int new_index = partition(shapes, start, end, dimension, cut_pos, swap_count);

    if (swap_count == 0 || new_index == start || new_index == end) {
        new_index = start + (end - start) / 2;
    }
    return new bvh_node(
            bounding_box,
            build_bvh_tlas(shapes, start, new_index, count),
            build_bvh_tlas(shapes, new_index, end, count)
    );
}


bool bvh_intersect_new(bvh_node* node, parser::Vec3f &camera_position, parser::Vec3f &ray_direction,
                   hit_record &rec, std::vector<std::shared_ptr<shape>> &shapes, std::vector<bvh_node*> &blas_hiers,parser::Vec3i &tlas_counts,
                   std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas, Eigen::Matrix4f &last_transformation_matrix,std::vector<std::shared_ptr<sphere>> &spheres,
                   std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes, parser::Material &current_material){


    if(node == nullptr || !node->bounding_box.does_intersect(camera_position, ray_direction)){

        return false;
    }


    if (node->is_leaf) {
        bool intersect = false;

        if (node->primitive_id != -1) {
            Eigen::Matrix4f current_transformation_matrix = node->bounding_box.get_transformation_matrix();
            Eigen::Matrix4f inverse_transformation_matrix = node->bounding_box.get_transformation_matrix().inverse();


            Eigen::Vector4f camera_4;
            camera_4 << camera_position.x, camera_position.y,camera_position.z, 1.0;
            camera_4 = inverse_transformation_matrix*camera_4;

            if(camera_4(3) != 1.0){
                camera_4 = camera_4/camera_4(3);
            }

            Eigen::Vector4f ray_4;
            ray_4 << ray_direction.x, ray_direction.y,ray_direction.z, 0.0;
            ray_4 = inverse_transformation_matrix*ray_4;

            parser::Vec3f new_cam_pos{camera_4(0), camera_4(1), camera_4(2)};
            parser::Vec3f new_ray_dir{ray_4(0), ray_4(1), ray_4(2)};
            new_ray_dir = normalize_vector(new_ray_dir);

            parser::Material new_material = node->bounding_box.get_material_struct();

            intersect = bvh_intersect_new(blas_hiers[node->bounding_box.object_index_in_tlas_vector], new_cam_pos, new_ray_dir,
                                              rec, primitives_inside_blas[node->bounding_box.object_index_in_tlas_vector], blas_hiers,tlas_counts,
                                              primitives_inside_blas,current_transformation_matrix,spheres, triangles, meshes, new_material);


        }else{
            ray_direction = normalize_vector(ray_direction);
            // inverse cam and ray to find actual t variable******************************************************************************
            Eigen::Vector4f camera_last_4;
            camera_last_4 << camera_position.x, camera_position.y,camera_position.z, 1.0;
            camera_last_4 = last_transformation_matrix*camera_last_4;

            if(camera_last_4(3) != 1.0){
                camera_last_4 = camera_last_4/camera_last_4(3);
            }
            parser::Vec3f before_cam_pos{camera_last_4(0), camera_last_4(1), camera_last_4(2)};


            Eigen::Vector4f ray_last_4;
            ray_last_4 << ray_direction.x, ray_direction.y,ray_direction.z, 0.0;
            ray_last_4 = last_transformation_matrix*ray_last_4;
            parser::Vec3f last_ray_dir{ray_last_4(0), ray_last_4(1), ray_last_4(2)};
            last_ray_dir = normalize_vector(last_ray_dir);

            //**************************************************************************************************************************************
            int primitive_id_hold = node->blas_id;
            float t_variable_hold = shapes[primitive_id_hold]->get_intersection_parameter(camera_position, ray_direction);

            // normal transformation and point transform****************************************************************************************************
            parser::Vec3f normal_calculation_point = vector_add(camera_position,
                                                                vector_multiply(ray_direction, t_variable_hold));

            //****************************************************************************************************************

            Eigen::Vector4f intersected_point_4;
            intersected_point_4 << normal_calculation_point.x, normal_calculation_point.y,normal_calculation_point.z, 1.0;
            intersected_point_4 = last_transformation_matrix*intersected_point_4;

            if(intersected_point_4(3) != 1.0){
                intersected_point_4 = intersected_point_4/intersected_point_4(3);
            }

            parser::Vec3f intersected_last{intersected_point_4(0), intersected_point_4(1), intersected_point_4(2)};
            parser::Vec3f t_d = vector_subtract(intersected_last, before_cam_pos);
            float t_og = get_vector_magnitude(vector_subtract(intersected_last, before_cam_pos));

            //****************************************************************************************************************

            parser::Vec3f untransformed_normal = shapes[primitive_id_hold]->get_normal(normal_calculation_point);

            auto normal_transformation_matrix_4x4 = last_transformation_matrix;

            Eigen::Vector4f intersection_transformation_holder;
            intersection_transformation_holder<< normal_calculation_point.x, normal_calculation_point.y, normal_calculation_point.z, 1.0;

            intersection_transformation_holder = normal_transformation_matrix_4x4* intersection_transformation_holder;

            if(intersection_transformation_holder(3) != 1.0){
                intersection_transformation_holder = intersection_transformation_holder/intersection_transformation_holder(3);
            }

            parser::Vec3f intersected_point_holder = parser::Vec3f{intersection_transformation_holder(0),
                                                                   intersection_transformation_holder(1),
                                                                   intersection_transformation_holder(2)};


            //take normal to world***************************************************************************
            Eigen::Matrix3f normal_transformation_matrix_3x3;
            normal_transformation_matrix_3x3 << normal_transformation_matrix_4x4(0,0), normal_transformation_matrix_4x4(0,1), normal_transformation_matrix_4x4(0,2),
            normal_transformation_matrix_4x4(1,0), normal_transformation_matrix_4x4(1,1), normal_transformation_matrix_4x4(1,2),
            normal_transformation_matrix_4x4(2,0), normal_transformation_matrix_4x4(2,1), normal_transformation_matrix_4x4(2,2);


            normal_transformation_matrix_3x3 = normal_transformation_matrix_3x3.inverse().transpose();

            Eigen::Vector3f normal_transformation_holder;
            normal_transformation_holder<< untransformed_normal.x, untransformed_normal.y, untransformed_normal.z;

            normal_transformation_holder = normal_transformation_matrix_3x3* normal_transformation_holder;

            parser::Vec3f transformed_normal{normal_transformation_holder(0),
                                                 normal_transformation_holder(1),
                                                 normal_transformation_holder(2)};

            transformed_normal = normalize_vector(transformed_normal);
            //normal over*******************************************************************************
            if(t_og > 0.0){
                rec.material = current_material;
                rec.normal = transformed_normal;
                rec.intersected_point = intersected_point_holder;
                rec.primitive_id = node->blas_id;

                if(rec.primitive_id == 0){
                    rec.color = parser::Vec3f{0,250,0};
                }else if(rec.primitive_id == 1){
                    rec.color = parser::Vec3f{250,0,0};
                }else if(rec.primitive_id == 2){
                    rec.color = parser::Vec3f{0,0,250};
                }

                rec.t_variable = t_og;
                rec.intersected_shape = shapes[rec.primitive_id];
                intersect = true;
            }


        }

        return intersect;
    }


    hit_record rec1, rec2;

    rec.t_variable = INFINITY;

    bool hit_left = bvh_intersect_new(node->left, camera_position, ray_direction, rec1, shapes,
                                      blas_hiers,tlas_counts,primitives_inside_blas, last_transformation_matrix,spheres, triangles, meshes,current_material);


    bool hit_right = bvh_intersect_new(node->right, camera_position, ray_direction, rec2, shapes,
                                       blas_hiers, tlas_counts,primitives_inside_blas, last_transformation_matrix,spheres, triangles, meshes,current_material);


    if(hit_left){

            rec = rec1;

    }

    if(hit_right){

        if (rec2.t_variable < rec.t_variable) {
            rec = rec2;
        }
    }

    if(hit_left || hit_right){
        //std::cout<<rec.t_variable<<std::endl;
    }

    return (hit_left || hit_right);
}



parser::Vec3f get_new_normalmap(parser::Vec3f &normal, parser::Vec3f &intersected_point ,std::shared_ptr<shape> &intersected_shape, texture &tex){


    parser::Vec3f read_tex_value = intersected_shape->get_texture_values(tex,intersected_point);


    if(read_tex_value.x < 0.0){
        read_tex_value.x = 0.0;
    }

    if(read_tex_value.y < 0.0){
        read_tex_value.y = 0.0;
    }

    if(read_tex_value.z < 0.0){
        read_tex_value.z = 0.0;
    }
    read_tex_value = vector_subtract(vector_multiply(read_tex_value, 2.0f / 255.0f), parser::Vec3f{1.0,1.0,1.0});

    read_tex_value = normalize_vector(read_tex_value);

    parser::Vec3f tangent_vec{0,0,0};
    parser::Vec3f bitangent_vec{0,0,0};

    parser::Vec3f result;
    if(intersected_shape->get_shape_type() == "triangle"){
        std::vector<parser::Vec3f> world_corners = intersected_shape->get_corner_world_coordinates();
        std::vector<parser::Vec2f> tex_corners = intersected_shape->get_corner_tex_coordinates();

        parser::Vec3f edge1 = vector_subtract(world_corners[1], world_corners[0]);
        parser::Vec3f edge2 = vector_subtract(world_corners[2], world_corners[1]);

        parser::Vec2f delta_uv1 = vector_subtract_2(tex_corners[1], tex_corners[0]);
        parser::Vec2f delta_uv2 = vector_subtract_2(tex_corners[2], tex_corners[1]);


        float div = 1.0f / (delta_uv1.u * delta_uv2.v - delta_uv2.u * delta_uv1.v);

        tangent_vec.x = div * (delta_uv2.v * edge1.x - delta_uv1.v * edge2.x);
        tangent_vec.y = div * (delta_uv2.v * edge1.y - delta_uv1.v * edge2.y);
        tangent_vec.z = div * (delta_uv2.v * edge1.z - delta_uv1.v * edge2.z);

        tangent_vec = normalize_vector(tangent_vec);

        bitangent_vec.x = div * (-delta_uv2.u * edge1.x + delta_uv1.u * edge2.x);
        bitangent_vec.y = div * (-delta_uv2.u * edge1.y + delta_uv1.u * edge2.y);
        bitangent_vec.z = div * (-delta_uv2.u * edge1.z + delta_uv1.u * edge2.z);


        bitangent_vec = normalize_vector(bitangent_vec);
        parser::Vec3f cross_normal = normalize_vector(cross_product(bitangent_vec, tangent_vec));


        if(dot_product(normal, cross_normal) < 0){
            cross_normal = normalize_vector(cross_product(tangent_vec, bitangent_vec));
        }

        parser::Vec3f hold_normal{0,0,0};

        hold_normal.x = read_tex_value.x * tangent_vec.x +
                        read_tex_value.y * bitangent_vec.x +
                        read_tex_value.z * cross_normal.x;

        hold_normal.y = read_tex_value.x * tangent_vec.y +
                        read_tex_value.y * bitangent_vec.y +
                        read_tex_value.z * cross_normal.y;

        hold_normal.z = read_tex_value.x * tangent_vec.z +
                        read_tex_value.y * bitangent_vec.z +
                        read_tex_value.z * cross_normal.z;



        result = normalize_vector(hold_normal);



    }else if(intersected_shape->get_shape_type() == "sphere"){
        float radius = intersected_shape->get_radius();
        parser::Vec2f phi_theta = intersected_shape->get_phi_theta(intersected_point);

        float got_u = ((-phi_theta.u + M_PI)/(2*M_PI));
        float got_v = (phi_theta.v / M_PI);

        float x = radius * sin(got_v * M_PI) * cos(M_PI - got_u * 2*M_PI);
        float y = radius * cos(got_v * M_PI);
        float z = radius * sin(got_v * M_PI) * sin(M_PI - got_u * 2*M_PI);

        tangent_vec.x = 2.0 * M_PI * z;
        tangent_vec.y = 0.0;
        tangent_vec.z = -2.0 * M_PI * x;

        tangent_vec = normalize_vector(tangent_vec);

        bitangent_vec.x = M_PI * y * cos(phi_theta.u);
        bitangent_vec.y = -radius * M_PI * sin(phi_theta.v);
        bitangent_vec.z = M_PI * y * sin(phi_theta.u);

        bitangent_vec = normalize_vector(bitangent_vec);

        parser::Vec3f cross_normal = normalize_vector(cross_product(bitangent_vec, tangent_vec));

        if(dot_product(normal, cross_normal) < 0){
            cross_normal = normalize_vector(cross_product(vector_multiply(bitangent_vec,-1), vector_multiply(tangent_vec,1)));
            bitangent_vec = vector_multiply(bitangent_vec, -1);
        }

        parser::Vec3f hold_normal{0,0,0};

        hold_normal.x = read_tex_value.x * tangent_vec.x +
                        read_tex_value.y * bitangent_vec.x +
                        read_tex_value.z * cross_normal.x;

        hold_normal.y = read_tex_value.x * tangent_vec.y +
                        read_tex_value.y * bitangent_vec.y +
                        read_tex_value.z * cross_normal.y;

        hold_normal.z = read_tex_value.x * tangent_vec.z +
                        read_tex_value.y * bitangent_vec.z +
                        read_tex_value.z * cross_normal.z;



        result = normalize_vector(hold_normal);


    }

    return result;

}


parser::Vec3f get_new_bumpmap(parser::Vec3f &normal, parser::Vec3f &intersected_point ,std::shared_ptr<shape> &intersected_shape, texture &tex){

    parser::Vec3f result;
    //image bump
    if(tex.texture_type == 0){
        //uv values of original intersection point
        parser::Vec2f uv_coordinates_intersection = intersected_shape->get_uv(intersected_point);

        float delta_u = 1.0/ tex.width;
        float delta_v = 1.0/ tex.height;
        //delta u and v to follow

        //original texture value(main)
        parser::Vec3f read_tex_value = tex.get_texture_value_bilinear(uv_coordinates_intersection);
        read_tex_value = vector_multiply(read_tex_value, 1.0f / tex.normalizer);

        float height = (read_tex_value.x + read_tex_value.y + read_tex_value.z) * 2.0;


        //u follow texture value
        parser::Vec2f u_int_point;
        u_int_point.u = uv_coordinates_intersection.u + delta_u;
        u_int_point.v = uv_coordinates_intersection.v;
        parser::Vec3f u_tex_value = tex.get_texture_value_bilinear(u_int_point);
        u_tex_value = vector_multiply(u_tex_value, 1.0f / tex.normalizer);

        float height_u = (u_tex_value.x + u_tex_value.y + u_tex_value.z) * 2.0;


        //v follow texture value
        parser::Vec2f v_int_point;
        v_int_point.u = uv_coordinates_intersection.u;
        v_int_point.v = uv_coordinates_intersection.v + delta_v;
        parser::Vec3f v_tex_value = tex.get_texture_value_bilinear(v_int_point);
        v_tex_value = vector_multiply(v_tex_value, 1.0f / tex.normalizer);

        float height_v = (v_tex_value.x + v_tex_value.y + v_tex_value.z) * 2.0;

        //*****************height value getting is done now*********************************
        //*****************move onto tangent bitangent**************************************

        parser::Vec3f tangent_vec;
        parser::Vec3f bitangent_vec;

        if(intersected_shape->get_shape_type() == "sphere"){
            float radius = intersected_shape->get_radius();
            parser::Vec2f phi_theta = intersected_shape->get_phi_theta(intersected_point);


            float x = radius * sin(uv_coordinates_intersection.v * M_PI) * cos(M_PI - uv_coordinates_intersection.u * 2*M_PI);
            float y = radius * cos(uv_coordinates_intersection.v * M_PI);
            float z = radius * sin(uv_coordinates_intersection.v * M_PI) * sin(M_PI - uv_coordinates_intersection.u * 2*M_PI);


            tangent_vec.x = 2.0 * M_PI * z;
            tangent_vec.y = 0.0;
            tangent_vec.z = -2.0 * M_PI * x;

            tangent_vec = normalize_vector(tangent_vec);


            bitangent_vec.x = M_PI * y * cos(phi_theta.u);
            bitangent_vec.y = -radius * M_PI *sin(phi_theta.v);
            bitangent_vec.z = M_PI * y * sin(phi_theta.u);

            bitangent_vec = normalize_vector(bitangent_vec);
        }else if(intersected_shape->get_shape_type() == "triangle"){
            std::vector<parser::Vec3f> world_corners = intersected_shape->get_corner_world_coordinates();
            std::vector<parser::Vec2f> tex_corners = intersected_shape->get_corner_tex_coordinates();

            parser::Vec3f edge1 = vector_subtract(world_corners[1], world_corners[0]);
            parser::Vec3f edge2 = vector_subtract(world_corners[2], world_corners[1]);

            parser::Vec2f delta_uv1 = vector_subtract_2(tex_corners[1], tex_corners[0]);
            parser::Vec2f delta_uv2 = vector_subtract_2(tex_corners[2], tex_corners[1]);


            float div = 1.0f / (delta_uv1.u * delta_uv2.v - delta_uv2.u * delta_uv1.v);

            tangent_vec.x = div * (delta_uv2.v * edge1.x - delta_uv1.v * edge2.x);
            tangent_vec.y = div * (delta_uv2.v * edge1.y - delta_uv1.v * edge2.y);
            tangent_vec.z = div * (delta_uv2.v * edge1.z - delta_uv1.v * edge2.z);

            tangent_vec = normalize_vector(tangent_vec);

            bitangent_vec.x = div * (-delta_uv2.u * edge1.x + delta_uv1.u * edge2.x);
            bitangent_vec.y = div * (-delta_uv2.u * edge1.y + delta_uv1.u * edge2.y);
            bitangent_vec.z = div * (-delta_uv2.u * edge1.z + delta_uv1.u * edge2.z);


            bitangent_vec = normalize_vector(bitangent_vec);
        }


        //get height partial derivatives
        float delta_height_u = (height_u - height) * tex.bump_factor;
        float delta_height_v = (height_v - height) * tex.bump_factor;

        //get normal
        parser::Vec3f hold_normal = normalize_vector(cross_product(bitangent_vec, tangent_vec));

        if(dot_product(normal, hold_normal) < 0){
            //std::cout<<"opp"<<std::endl;
            hold_normal = normalize_vector(cross_product(vector_multiply(bitangent_vec,1), vector_multiply(tangent_vec,-1)));
            tangent_vec = vector_multiply(tangent_vec, -1);
        }
        //calculate q vectors
        parser::Vec3f q_du = vector_add(tangent_vec, vector_multiply(hold_normal, delta_height_u));
        parser::Vec3f q_dv = vector_add(bitangent_vec, vector_multiply(hold_normal, delta_height_v));

        //get cross product and the final vector
        parser::Vec3f final_normal = cross_product(q_dv, q_du);

        if (get_vector_magnitude(final_normal) > 1e-6) {
            result = normalize_vector(final_normal);
        } else {
            result = normal;
        }


        // perlin bump
    }else if(tex.texture_type == 1){

        float epsilon = 0.001;

        //original perlin height
        parser::Vec3f perlin_height_original = tex.get_texture_value_perlin(intersected_point);
        perlin_height_original = vector_multiply(perlin_height_original, 1.0f / tex.normalizer);

        float original_height = (perlin_height_original.x + perlin_height_original.y + perlin_height_original.z);

        //x forward perlin height
        parser::Vec3f x_forward_point{intersected_point.x + epsilon, intersected_point.y, intersected_point.z};
        parser::Vec3f perlin_height_x = tex.get_texture_value_perlin(x_forward_point);
        perlin_height_x = vector_multiply(perlin_height_x, 1.0f / tex.normalizer);

        float x_height = (perlin_height_x.x + perlin_height_x.y + perlin_height_x.z);

        //y forward perlin height
        parser::Vec3f y_forward_point{intersected_point.x, intersected_point.y + epsilon, intersected_point.z};
        parser::Vec3f perlin_height_y = tex.get_texture_value_perlin(y_forward_point);
        perlin_height_y = vector_multiply(perlin_height_y, 1.0f / tex.normalizer);

        float y_height = (perlin_height_y.x + perlin_height_y.y + perlin_height_y.z);

        //z forward perlin height
        parser::Vec3f z_forward_point{intersected_point.x, intersected_point.y, intersected_point.z + epsilon};
        parser::Vec3f perlin_height_z = tex.get_texture_value_perlin(z_forward_point);
        perlin_height_z = vector_multiply(perlin_height_z, 1.0f / tex.normalizer);

        float z_height = (perlin_height_z.x + perlin_height_z.y + perlin_height_z.z);

        parser::Vec3f perlin_gradient_vector;



        // get tangent and bitangent *****************************************************************************************
        parser::Vec3f tangent_vec;
        parser::Vec3f bitangent_vec;

        if(intersected_shape->get_shape_type() == "sphere"){
            perlin_gradient_vector.x = 55*(x_height - original_height) / epsilon;
            perlin_gradient_vector.y = 55*(y_height - original_height) / epsilon;
            perlin_gradient_vector.z = 55*(z_height - original_height) / epsilon;
            float radius = intersected_shape->get_radius();
            parser::Vec2f phi_theta = intersected_shape->get_phi_theta(intersected_point);
            parser::Vec2f uv_coordinates_intersection = intersected_shape->get_uv(intersected_point);

            float x = radius * sin(uv_coordinates_intersection.v * M_PI) * cos(M_PI - uv_coordinates_intersection.u * 2*M_PI);
            float y = radius * cos(uv_coordinates_intersection.v * M_PI);
            float z = radius * sin(uv_coordinates_intersection.v * M_PI) * sin(M_PI - uv_coordinates_intersection.u * 2*M_PI);


            tangent_vec.x = 2.0 * M_PI * z;
            tangent_vec.y = 0.0;
            tangent_vec.z = -2.0 * M_PI * x;

            tangent_vec = normalize_vector(tangent_vec);


            bitangent_vec.x = M_PI * y * cos(phi_theta.u);
            bitangent_vec.y = -radius * M_PI *sin(phi_theta.v);
            bitangent_vec.z = M_PI * y * sin(phi_theta.u);

            bitangent_vec = normalize_vector(bitangent_vec);
        }else if(intersected_shape->get_shape_type() == "triangle"){
            perlin_gradient_vector.x = 10*(x_height - original_height) / epsilon;
            perlin_gradient_vector.y = 10*(y_height - original_height) / epsilon;
            perlin_gradient_vector.z = 10*(z_height - original_height) / epsilon;

            std::vector<parser::Vec3f> world_corners = intersected_shape->get_corner_world_coordinates();
            std::vector<parser::Vec2f> tex_corners = intersected_shape->get_corner_tex_coordinates();

            parser::Vec3f edge1 = vector_subtract(world_corners[1], world_corners[0]);
            parser::Vec3f edge2 = vector_subtract(world_corners[2], world_corners[1]);

            parser::Vec2f delta_uv1 = vector_subtract_2(tex_corners[1], tex_corners[0]);
            parser::Vec2f delta_uv2 = vector_subtract_2(tex_corners[2], tex_corners[1]);


            float div = 1.0f / (delta_uv1.u * delta_uv2.v - delta_uv2.u * delta_uv1.v);

            tangent_vec.x = div * (delta_uv2.v * edge1.x - delta_uv1.v * edge2.x);
            tangent_vec.y = div * (delta_uv2.v * edge1.y - delta_uv1.v * edge2.y);
            tangent_vec.z = div * (delta_uv2.v * edge1.z - delta_uv1.v * edge2.z);

            tangent_vec = normalize_vector(tangent_vec);

            bitangent_vec.x = div * (-delta_uv2.u * edge1.x + delta_uv1.u * edge2.x);
            bitangent_vec.y = div * (-delta_uv2.u * edge1.y + delta_uv1.u * edge2.y);
            bitangent_vec.z = div * (-delta_uv2.u * edge1.z + delta_uv1.u * edge2.z);


            bitangent_vec = normalize_vector(bitangent_vec);
        }

        //*******************************************************************************************************************


        parser::Vec3f hold_normal = normalize_vector(cross_product(bitangent_vec, tangent_vec));
        if(dot_product(normal, hold_normal) < 0){
            hold_normal = normalize_vector(cross_product(vector_multiply(bitangent_vec,-1), vector_multiply(tangent_vec,1)));
            bitangent_vec = vector_multiply(bitangent_vec, -1);
        }
        parser::Vec3f g_parallel = vector_multiply(hold_normal,dot_product(perlin_gradient_vector, hold_normal));
        parser::Vec3f surface_gradiant = (vector_subtract(perlin_gradient_vector, g_parallel));

        parser::Vec3f new_normal = vector_subtract(hold_normal,surface_gradiant);


        result = normalize_vector(new_normal);


    }
    return result;
}




bool bvh_intersect_mb(float time,float near_distance, bool is_camera, bool ignore_np,parser::Vec3f &gaze, bvh_node* node, parser::Vec3f &camera_position, parser::Vec3f &ray_direction,
                       hit_record &rec, std::vector<std::shared_ptr<shape>> &shapes, std::vector<bvh_node*> &blas_hiers,parser::Vec3i &tlas_counts,
                       std::vector<std::vector<std::shared_ptr<shape>>> &primitives_inside_blas, Eigen::Matrix4f last_transformation_matrix,std::vector<std::shared_ptr<sphere>> &spheres,
                       std::vector<std::shared_ptr<triangle>> &triangles, std::vector<std::shared_ptr<mesh>> &meshes, parser::Material &current_material,
                       bool is_light, bool is_cloud,parser::Vec3f &current_radiance,std::vector<texture> &textures,std::vector<int> &current_tex_ids){


    if(node == nullptr || !node->bounding_box.does_intersect(camera_position, ray_direction)){

        return false;
    }

    if (node->is_leaf) {
        bool intersect = false;

        if (node->primitive_id != -1) {

            Eigen::Vector4f camera_4;
            camera_4 << camera_position.x, camera_position.y,camera_position.z, 1.0;
            Eigen::Vector4f ray_4;
            ray_4 << ray_direction.x, ray_direction.y,ray_direction.z, 0.0;

            Eigen::Matrix4f motion_blur_matrix = node->bounding_box.motion_blur_matrix;

            if(node->bounding_box.motion_blur_matrix != Eigen::Matrix4f::Identity()){


                motion_blur_matrix(0,3) *= time;
                motion_blur_matrix(1,3) *= time;
                motion_blur_matrix(2,3) *= time;
                Eigen::Matrix4f inverse_motion_blur = motion_blur_matrix.inverse();
                camera_4 = inverse_motion_blur*camera_4;

                if(camera_4(3) != 1.0){
                    camera_4 = camera_4/camera_4(3);
                }

                ray_4 = inverse_motion_blur*ray_4;

            }

            Eigen::Matrix4f current_transformation_matrix = node->bounding_box.get_transformation_matrix();
            Eigen::Matrix4f inverse_transformation_matrix = node->bounding_box.get_transformation_matrix().inverse();


            camera_4 = inverse_transformation_matrix*camera_4;

            if(camera_4(3) != 1.0){
                camera_4 = camera_4/camera_4(3);
            }

            ray_4 = inverse_transformation_matrix*ray_4;

            current_transformation_matrix = motion_blur_matrix*current_transformation_matrix;


            parser::Vec3f new_cam_pos{camera_4(0), camera_4(1), camera_4(2)};
            parser::Vec3f new_ray_dir{ray_4(0), ray_4(1), ray_4(2)};
            new_ray_dir = normalize_vector(new_ray_dir);

            parser::Material new_material = node->bounding_box.get_material_struct();
            std::vector<int> new_tex_ids = node->bounding_box.texture_ids;

            parser::Vec3f new_radiance{0,0,0};
            bool is_light_current = node->bounding_box.get_is_light();
            bool is_cloud_current = node->bounding_box.get_is_cloud();

            if(node->bounding_box.get_is_light()){
                new_radiance = node->bounding_box.radiance;
            }


            intersect = bvh_intersect_mb(time,near_distance, is_camera,ignore_np,gaze,blas_hiers[node->bounding_box.object_index_in_tlas_vector], new_cam_pos, new_ray_dir,
                                          rec, primitives_inside_blas[node->bounding_box.object_index_in_tlas_vector], blas_hiers,tlas_counts,
                                          primitives_inside_blas,current_transformation_matrix,spheres, triangles, meshes, new_material,
                                         is_light_current, is_cloud_current,new_radiance,textures,new_tex_ids);


        }else{


            ray_direction = normalize_vector(ray_direction);
            // inverse cam and ray to find actual t variable******************************************************************************
            Eigen::Vector4f camera_last_4;
            camera_last_4 << camera_position.x, camera_position.y,camera_position.z, 1.0;
            camera_last_4 = last_transformation_matrix*camera_last_4;

            if(camera_last_4(3) != 1.0){
                camera_last_4 = camera_last_4/camera_last_4(3);
            }
            parser::Vec3f before_cam_pos{camera_last_4(0), camera_last_4(1), camera_last_4(2)};


            Eigen::Vector4f ray_last_4;
            ray_last_4 << ray_direction.x, ray_direction.y,ray_direction.z, 0.0;
            ray_last_4 = last_transformation_matrix*ray_last_4;
            parser::Vec3f last_ray_dir{ray_last_4(0), ray_last_4(1), ray_last_4(2)};
            last_ray_dir = normalize_vector(last_ray_dir);

            //**************************************************************************************************************************************
            int primitive_id_hold = node->blas_id;
            float t_variable_hold = shapes[primitive_id_hold]->get_intersection_parameter(camera_position, ray_direction);

            // normal transformation and point transform****************************************************************************************************
            parser::Vec3f normal_calculation_point = vector_add(camera_position,
                                                                vector_multiply(ray_direction, t_variable_hold));

            //****************************************************************************************************************

            Eigen::Vector4f intersected_point_4;
            intersected_point_4 << normal_calculation_point.x, normal_calculation_point.y,normal_calculation_point.z, 1.0;
            intersected_point_4 = last_transformation_matrix*intersected_point_4;

            if(intersected_point_4(3) != 1.0){
                intersected_point_4 = intersected_point_4/intersected_point_4(3);
            }

            parser::Vec3f intersected_last{intersected_point_4(0), intersected_point_4(1), intersected_point_4(2)};
            parser::Vec3f t_d = vector_subtract(intersected_last, before_cam_pos);
            float t_og = get_vector_magnitude(vector_subtract(intersected_last, before_cam_pos));

            //****************************************************************************************************************

            parser::Vec3f untransformed_normal = shapes[primitive_id_hold]->get_normal(normal_calculation_point);


            if(textures.size() > 0){
                for(auto &tex_id: current_tex_ids){
                    //normal mapping
                    if(textures[tex_id].decal_mode == 4){
                        untransformed_normal = get_new_normalmap(untransformed_normal, normal_calculation_point, shapes[node->blas_id], textures[tex_id]);

                        //bump mapping
                    }else if(textures[tex_id].decal_mode == 5){
                        untransformed_normal = get_new_bumpmap(untransformed_normal, normal_calculation_point, shapes[node->blas_id], textures[tex_id]);
                    }

                }
            }

            rec.kd = current_material.diffuse;
            rec.ks = current_material.specular;
            if(textures.size() > 0){

                for(auto &tex_id: current_tex_ids){
                    if(textures[tex_id].texture_type == 0){
                        //replace_kd
                        if(textures[tex_id].decal_mode == 0){
                            rec.kd = shapes[node->blas_id]->get_texture_values(textures[tex_id],normal_calculation_point);
                            rec.kd.x /= textures[tex_id].normalizer;
                            rec.kd.y /= textures[tex_id].normalizer;
                            rec.kd.z /= textures[tex_id].normalizer;
                            //blend_kd
                        }else if(textures[tex_id].decal_mode == 1){
                            rec.kd = shapes[node->blas_id]->get_texture_values(textures[tex_id],normal_calculation_point);
                            rec.kd.x /= textures[tex_id].normalizer;
                            rec.kd.y /= textures[tex_id].normalizer;
                            rec.kd.z /= textures[tex_id].normalizer;
                            rec.kd = vector_add(current_material.diffuse, rec.kd);
                            rec.kd.x /= 2.0;
                            rec.kd.y /= 2.0;
                            rec.kd.z /= 2.0;
                            //replace_ks
                        }else if(textures[tex_id].decal_mode == 2) {
                            rec.ks = shapes[node->blas_id]->get_texture_values(textures[tex_id], normal_calculation_point);
                            rec.ks.x /= textures[tex_id].normalizer;
                            rec.ks.y /= textures[tex_id].normalizer;
                            rec.ks.z /= textures[tex_id].normalizer;
                        }else if(textures[tex_id].decal_mode == 6){
                            rec.color = shapes[node->blas_id]->get_texture_values(textures[tex_id], normal_calculation_point);
                        }

                    }else if(textures[tex_id].texture_type == 1){
                        if(textures[tex_id].decal_mode == 0){
                            rec.kd = textures[tex_id].get_texture_value_procedural(normal_calculation_point);
                            //blend_kd
                        }else if(textures[tex_id].decal_mode == 1){
                            rec.kd = vector_add(current_material.diffuse,textures[tex_id].get_texture_value_procedural(normal_calculation_point));
                            rec.kd.x /= 2.0;
                            rec.kd.y /= 2.0;
                            rec.kd.z /= 2.0;
                            //replace_ks
                        }else if(textures[tex_id].decal_mode == 2) {
                            rec.kd = textures[tex_id].get_texture_value_procedural(normal_calculation_point);
                        }

                    }else if(textures[tex_id].texture_type == 2){
                        if(textures[tex_id].decal_mode == 0){
                            rec.kd = textures[tex_id].get_texture_value_procedural(normal_calculation_point);
                            //blend_kd
                        }else if(textures[tex_id].decal_mode == 1){
                            rec.kd = textures[tex_id].get_texture_value_procedural(normal_calculation_point);
                            rec.kd.x /= 2.0;
                            rec.kd.y /= 2.0;
                            rec.kd.z /= 2.0;
                            //replace_ks
                        }else if(textures[tex_id].decal_mode == 2) {
                            rec.kd = textures[tex_id].get_texture_value_procedural(normal_calculation_point);
                        }
                    }

                }
            }



            auto normal_transformation_matrix_4x4 = last_transformation_matrix;

            Eigen::Vector4f intersection_transformation_holder;
            intersection_transformation_holder<< normal_calculation_point.x, normal_calculation_point.y, normal_calculation_point.z, 1.0;

            intersection_transformation_holder = normal_transformation_matrix_4x4* intersection_transformation_holder;

            if(intersection_transformation_holder(3) != 1.0){
                intersection_transformation_holder = intersection_transformation_holder/intersection_transformation_holder(3);
            }

            parser::Vec3f intersected_point_holder = parser::Vec3f{intersection_transformation_holder(0),
                                                                   intersection_transformation_holder(1),
                                                                   intersection_transformation_holder(2)};


            //take normal to world***************************************************************************
            Eigen::Matrix3f normal_transformation_matrix_3x3;
            normal_transformation_matrix_3x3 << normal_transformation_matrix_4x4(0,0), normal_transformation_matrix_4x4(0,1), normal_transformation_matrix_4x4(0,2),
                    normal_transformation_matrix_4x4(1,0), normal_transformation_matrix_4x4(1,1), normal_transformation_matrix_4x4(1,2),
                    normal_transformation_matrix_4x4(2,0), normal_transformation_matrix_4x4(2,1), normal_transformation_matrix_4x4(2,2);


            normal_transformation_matrix_3x3 = normal_transformation_matrix_3x3.inverse().transpose();

            Eigen::Vector3f normal_transformation_holder;
            normal_transformation_holder<< untransformed_normal.x, untransformed_normal.y, untransformed_normal.z;

            normal_transformation_holder = normal_transformation_matrix_3x3* normal_transformation_holder;

            parser::Vec3f transformed_normal{normal_transformation_holder(0),
                                             normal_transformation_holder(1),
                                             normal_transformation_holder(2)};

            transformed_normal = normalize_vector(transformed_normal);
            //normal over*******************************************************************************
            if(t_og > 0.0){
                if(is_camera){
                    if(ignore_np || (dot_product(vector_multiply(last_ray_dir,t_og), gaze)>= near_distance)){
                        rec.material = current_material;
                        rec.radiance = current_radiance;
                        rec.is_light = is_light;
                        rec.is_cloud = is_cloud;

                        rec.normal = transformed_normal;
                        rec.intersected_point = intersected_point_holder;
                        rec.primitive_id = node->blas_id;

                        rec.tex_ids = current_tex_ids;
                        rec.t_variable = t_og;
                        rec.intersected_shape = shapes[rec.primitive_id];
                        intersect = true;
                    }
                }else{
                    rec.material = current_material;
                    rec.radiance = current_radiance;
                    rec.is_light = is_light;
                    rec.is_cloud = is_cloud;

                    rec.normal = transformed_normal;
                    rec.intersected_point = intersected_point_holder;
                    rec.primitive_id = node->blas_id;


                    rec.tex_ids = current_tex_ids;
                    rec.t_variable = t_og;
                    rec.intersected_shape = shapes[rec.primitive_id];
                    intersect = true;
                }

            }


        }

        return intersect;
    }


    hit_record rec1, rec2;

    rec.t_variable = INFINITY;


    bool hit_left = bvh_intersect_mb(time,near_distance,is_camera,ignore_np,gaze,node->left, camera_position, ray_direction, rec1, shapes,
                                      blas_hiers,tlas_counts,primitives_inside_blas, last_transformation_matrix,spheres, triangles, meshes,current_material,
                                      is_light, is_cloud,current_radiance,textures,current_tex_ids);


    bool hit_right = bvh_intersect_mb(time, near_distance,is_camera,ignore_np,gaze,node->right, camera_position, ray_direction, rec2, shapes,
                                       blas_hiers, tlas_counts,primitives_inside_blas, last_transformation_matrix,spheres, triangles, meshes,current_material,
                                       is_light, is_cloud,current_radiance,textures,current_tex_ids);


    if(hit_left){

        rec = rec1;

    }

    if(hit_right){

        if (rec2.t_variable < rec.t_variable) {
            rec = rec2;
        }
    }

    if(hit_left || hit_right){
        //std::cout<<rec.t_variable<<std::endl;
    }

    return (hit_left || hit_right);
}





int build_bvh_linearized(std::vector<std::shared_ptr<shape>> &shapes,
                         std::vector<bvh_node_linear> &nodes_linearized,
                         int start, int end) {

    bbox bounding_box(shapes, start, end);

    if (end - start <= 1) {
        size_t leaf_index = nodes_linearized.size();
        auto new_leaf_node = bvh_node_linear(bounding_box, start);
        nodes_linearized.push_back(new_leaf_node);
        return (int)leaf_index;
    }
    parser::Vec3f dimension_differences = bounding_box.get_dimension_differences();

    float biggest_dimension_difference = fmax(dimension_differences.x,
                                              fmax(dimension_differences.y, dimension_differences.z));

    int dimension;
    float cut_pos;
    std::vector<parser::Vec3f> corners = bounding_box.get_corners();
    if(fabs(biggest_dimension_difference - dimension_differences.x) < 0.00001){
        dimension = 0;
        cut_pos = (corners[0].x + corners[1].x) / 2.0;
    }else if(fabs(biggest_dimension_difference - dimension_differences.y) < 0.00001){
        dimension = 1;
        cut_pos = (corners[0].y + corners[1].y) / 2.0;
    }else if(fabs(biggest_dimension_difference - dimension_differences.z) < 0.00001){
        dimension = 2;
        cut_pos = (corners[0].z + corners[1].z) / 2.0;
    }

    int swap_count = 0;
    int new_index = partition(shapes, start, end, dimension, cut_pos, swap_count);

    if (swap_count == 0 || new_index == start || new_index == end) {
        new_index = start + (end - start) / 2;
    }
    size_t node_index = nodes_linearized.size();
    nodes_linearized.emplace_back(bvh_node_linear(bounding_box, -1,-1));
    auto new_node = bvh_node_linear(bounding_box,
                                  build_bvh_linearized(shapes, nodes_linearized,start, new_index),
                                  build_bvh_linearized(shapes, nodes_linearized, new_index, end));
    nodes_linearized[node_index] = new_node;
    return (int)node_index;
}

bool bvh_intersect_linear(std::vector<bvh_node_linear> &nodes, int index, parser::Vec3f &camera_position,
                          parser::Vec3f &ray_direction, hit_record &rec, std::vector<std::shared_ptr<shape>> &shapes){

    bvh_node_linear* current_node = &nodes[index];
    if(!current_node->bounding_box.does_intersect(camera_position, ray_direction)){

        return false;
    }

    if (current_node->is_leaf) {
        bool intersect = false;

        if (current_node->primitive_id != -1) {
            rec.primitive_id = current_node->primitive_id;
            rec.t_variable = shapes[rec.primitive_id]->get_intersection_parameter(camera_position, ray_direction);
            if(rec.t_variable > 0.0){
                intersect = true;
            }
        }

        return intersect;
    }
    hit_record rec1, rec2;

    rec.t_variable = INFINITY;

    bool hit_left = bvh_intersect_linear(nodes, current_node->left_index, camera_position, ray_direction, rec1, shapes);
    bool hit_right = bvh_intersect_linear(nodes, current_node->right_index, camera_position, ray_direction, rec2, shapes);

    if(hit_left){
        rec = rec1;
    }

    if(hit_right){
        rec = rec2.t_variable < rec.t_variable ? rec2 : rec;
    }

    return (hit_left || hit_right);
}

