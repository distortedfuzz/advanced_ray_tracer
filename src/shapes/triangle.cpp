#include "triangle.h"
#include "../math/math.h"
#include "../transformations/rotation.h"
#include "../transformations/scaling.h"
#include "../transformations/translation.h"

#include <iostream>

triangle::triangle(int id,
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
                   bool is_cloud): shape(id, material_id, materials, texture_ids, is_light, is_cloud,radiance, "triangle") {

    this->indices = indices;
    this->indices.x += vertex_offset;
    this->indices.y += vertex_offset;
    this->indices.z += vertex_offset;

    parser::Vec3f corner0;
    corner0.x = vertex_data[this->indices.x-1].x;
    corner0.y = vertex_data[this->indices.x-1].y;
    corner0.z = vertex_data[this->indices.x-1].z;


    parser::Vec3f corner1;
    corner1.x = vertex_data[this->indices.y-1].x;
    corner1.y = vertex_data[this->indices.y-1].y;
    corner1.z = vertex_data[this->indices.y-1].z;


    parser::Vec3f corner2;
    corner2.x = vertex_data[this->indices.z-1].x;
    corner2.y = vertex_data[this->indices.z-1].y;
    corner2.z = vertex_data[this->indices.z-1].z;

    this->transformation_matrix = calculate_transformation_matrix(transformations, translations, rotations, scalings, composites);

    corners.push_back(corner0);
    corners.push_back(corner1);
    corners.push_back(corner2);

    parser::Vec3f p1 = corners[0];
    parser::Vec3f p2 = corners[1];
    parser::Vec3f p3 = corners[2];

    center.x = (corner0.x + corner1.x + corner2.x)/3.0;
    center.y = (corner0.y + corner1.y + corner2.y)/3.0;
    center.z = (corner0.z + corner1.z + corner2.z)/3.0;


    if(tex_coord_data.size() > 0){
        parser::Vec2f corner0_tex;
        corner0_tex.u = tex_coord_data[indices.x-1 + texture_offset].u;
        corner0_tex.v = tex_coord_data[indices.x-1 + texture_offset].v;

        parser::Vec2f corner1_tex;
        corner1_tex.u = tex_coord_data[indices.y-1 + texture_offset].u;
        corner1_tex.v = tex_coord_data[indices.y-1 + texture_offset].v;

        parser::Vec2f corner2_tex;
        corner2_tex.u = tex_coord_data[indices.z-1 + texture_offset].u;
        corner2_tex.v = tex_coord_data[indices.z-1 + texture_offset].v;

        this->tex_coords.push_back(corner0_tex);
        this->tex_coords.push_back(corner1_tex);
        this->tex_coords.push_back(corner2_tex);
    }

    this->texture_ids = texture_ids;

    parser::Vec3f vec1 = vector_subtract(p1, p2);
    parser::Vec3f vec2 = vector_subtract(p3, p2);
    this->normal = vector_divide(cross_product(vec2, vec1), get_vector_magnitude(cross_product(vec2, vec1)));
}

triangle::triangle(std::vector<parser::Vec3f> &corners,
                   Eigen::Matrix4f &tr_matrix, int material_id, std::vector<int> &texture_ids,
                   std::vector<parser::Material> &materials,parser::Vec3f radiance,
                   bool is_light,
                   bool is_cloud):
                   shape(0, material_id, materials,texture_ids, is_light, is_cloud,radiance,"triangle"){


    for(auto &corner:corners){
        parser::Vec3f new_corner;
        new_corner.x = corner.x;
        new_corner.y = corner.y;
        new_corner.z = corner.z;
        this->corners.push_back(new_corner);
    }

    this->transformation_matrix = tr_matrix;

}


parser::Vec3f triangle::get_texture_values(texture &tex, parser::Vec3f &intersection_point){

    parser::Vec3f p1 = corners[0];
    parser::Vec3f p2 = corners[1];
    parser::Vec3f p3 = corners[2];

    float triangle_area = get_vector_magnitude(cross_product(vector_subtract(p2, p1), vector_subtract(p3,p1)))/2.0;

    float area1 = get_vector_magnitude(cross_product(vector_subtract(p2, intersection_point),
                                                     vector_subtract(p3,intersection_point)))/2.0;

    float area2 = get_vector_magnitude(cross_product(vector_subtract(p3, intersection_point),
                                                     vector_subtract(p1,intersection_point)))/2.0;

    float area3 = get_vector_magnitude(cross_product(vector_subtract(p1, intersection_point),
                                                     vector_subtract(p2,intersection_point)))/2.0;


    float alpha = area1 / triangle_area;
    float beta = area2 / triangle_area;
    float gamma = area3 / triangle_area;



    parser::Vec2f texture_coordinate;
    texture_coordinate.u = tex_coords[0].u*alpha + tex_coords[1].u*beta + tex_coords[2].u*gamma;
    texture_coordinate.v = tex_coords[0].v*alpha + tex_coords[1].v*beta + tex_coords[2].v*gamma;

    texture_coordinate.u = texture_coordinate.u - floor(texture_coordinate.u);
    texture_coordinate.v = texture_coordinate.v - floor(texture_coordinate.v);

    return tex.get_texture_value(texture_coordinate);
}

parser::Vec2f triangle::get_uv(parser::Vec3f &intersected_point){
    parser::Vec3f p1 = corners[0];
    parser::Vec3f p2 = corners[1];
    parser::Vec3f p3 = corners[2];

    float triangle_area = get_vector_magnitude(cross_product(vector_subtract(p2, p1), vector_subtract(p3,p1)))/2.0;

    float area1 = get_vector_magnitude(cross_product(vector_subtract(p2, intersected_point),
                                                     vector_subtract(p3,intersected_point)))/2.0;

    float area2 = get_vector_magnitude(cross_product(vector_subtract(p3, intersected_point),
                                                     vector_subtract(p1,intersected_point)))/2.0;

    float area3 = get_vector_magnitude(cross_product(vector_subtract(p1, intersected_point),
                                                     vector_subtract(p2,intersected_point)))/2.0;


    float alpha = area1 / triangle_area;
    float beta = area2 / triangle_area;
    float gamma = area3 / triangle_area;

    parser::Vec2f texture_coordinate;
    texture_coordinate.u = tex_coords[0].u*alpha + tex_coords[1].u*beta + tex_coords[2].u*gamma;
    texture_coordinate.v = tex_coords[0].v*alpha + tex_coords[1].v*beta + tex_coords[2].v*gamma;

    texture_coordinate.u = texture_coordinate.u - floor(texture_coordinate.u);
    texture_coordinate.v = texture_coordinate.v - floor(texture_coordinate.v);
    //std::cout<<texture_coordinate.u<<" "<<texture_coordinate.v<<std::endl;

    return texture_coordinate;
}



parser::Vec3i triangle::get_indices(){
    return this->indices;
}

std::vector<parser::Vec3f> triangle::get_corners() {
    return this->corners;
}

std::vector<float> triangle::get_min_max(){

    float max_x, min_x;
    float max_y, min_y;
    float max_z, min_z;

    for(int i = 0; i< corners.size(); i++){
        if(i == 0){
            max_x = corners[i].x;
            min_x = corners[i].x;

            max_y = corners[i].y;
            min_y = corners[i].y;

            max_z = corners[i].z;
            min_z = corners[i].z;
        }else{
            if(corners[i].x > max_x){
                max_x = corners[i].x;
            }
            if(corners[i].x < min_x){
                min_x = corners[i].x;
            }
            if(corners[i].y > max_y){
                max_y = corners[i].y;
            }
            if(corners[i].y < min_y){
                min_y = corners[i].y;
            }
            if(corners[i].z > max_z){
                max_z = corners[i].z;
            }
            if(corners[i].z < min_z){
                min_z = corners[i].z;
            }
        }
    }
    std::vector<float> min_max_points;
    //this caused problems without offset 1e-5 did not work with lobster
    min_max_points.push_back(max_x+0.00005);
    min_max_points.push_back(min_x-0.00005);
    min_max_points.push_back(max_y+0.00005);
    min_max_points.push_back(min_y-0.00005);
    min_max_points.push_back(max_z+0.00005);
    min_max_points.push_back(min_z-0.00005);

    return min_max_points;
}

parser::Vec3f triangle::get_normal(parser::Vec3f &point){

    return normal;
}

parser::Vec3f triangle::get_center(){

    return center;
}

Eigen::Matrix4f triangle::get_transformation_matrix(){
    return transformation_matrix;
}


float triangle::get_intersection_parameter(const parser::Vec3f &cam_position, const parser::Vec3f &ray_direction) {

    parser::Vec3f normalized_vec = normalize_vector(ray_direction);

    parser::Vec3f p1 = corners[0];
    parser::Vec3f p2 = corners[1];
    parser::Vec3f p3 = corners[2];

    float det_a = get_3x3_determinant(vector_subtract(p1, p2), vector_subtract(p1, p3), normalized_vec);

    float beta = get_3x3_determinant(vector_subtract(p1, cam_position), vector_subtract(p1, p3), normalized_vec)/ det_a;

    float gamma = get_3x3_determinant(vector_subtract(p1,p2), vector_subtract(p1, cam_position), normalized_vec)/ det_a;

    float parameter = get_3x3_determinant(vector_subtract(p1, p2), vector_subtract(p1, p3), vector_subtract(p1, cam_position))/ det_a;


    if(gamma>=-0.00001  && beta>=-0.00001 && (beta +gamma)<=1.00001 && parameter >0.00001){
        return parameter;
    }else{
        return 0.0;
    }

}

void triangle::apply_transformation() {

    for(auto &corner: corners){
        Eigen::Vector4f one_corner;
        one_corner << corner.x, corner.y,corner.z, 1.0;

        one_corner = transformation_matrix*one_corner;

        corner.x = one_corner(0);
        corner.y = one_corner(1);
        corner.z = one_corner(2);
    }

}



std::vector<parser::Vec3f> triangle::get_corner_world_coordinates(){
    return corners;
}


std::vector<parser::Vec2f> triangle::get_corner_tex_coordinates(){
    return tex_coords;
}