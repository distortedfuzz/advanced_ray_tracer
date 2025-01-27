#include <memory>
#include "mesh.h"
#include "../math/math.h"
#include "../transformations/rotation.h"
#include "../transformations/scaling.h"
#include "../transformations/translation.h"


mesh::mesh(int id,int material_id, std::vector<int> &texture_ids,
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
           bool is_cloud): shape(id, material_id, materials, texture_ids,is_light, is_cloud,radiance,"mesh") {


    this->transformation_matrix = calculate_transformation_matrix(transformations, translations, rotations, scalings, composites);
    this->id = id;
    this->faces = faces;
    std::cout<<"is cloud: "<<is_cloud<<std::endl;
    for(int i = 0; i < faces.size(); i++){


        this->faces[i].x += vertex_offset;
        this->faces[i].y += vertex_offset;
        this->faces[i].z += vertex_offset;

        parser::Vec3f corner1;
        parser::Vec2f tex_coord1;



        corner1 = vertex_data[this->faces[i].x - 1];
        if(!tex_coord_data.empty() && texture_ids.size() >0){
            tex_coord1 = tex_coord_data[faces[i].x-1 + texture_offset];
        }


        parser::Vec3f corner2;
        parser::Vec2f tex_coord2;

        corner2 = vertex_data[this->faces[i].y - 1];
        if(!tex_coord_data.empty()&& texture_ids.size() >0){
            tex_coord2 = tex_coord_data[faces[i].y-1 + texture_offset];
        }


        parser::Vec3f corner3;
        parser::Vec2f tex_coord3;

        corner3 = vertex_data[this->faces[i].z - 1];
        if(!tex_coord_data.empty()&& texture_ids.size() >0){
            tex_coord3 = tex_coord_data[faces[i].z-1 + texture_offset];
        }



        std::vector<parser::Vec3f> new_face_vertices;

        new_face_vertices.push_back(corner1);
        new_face_vertices.push_back(corner2);
        new_face_vertices.push_back(corner3);

        std::vector<parser::Vec2f> new_tex_vertices;

        new_tex_vertices.push_back(tex_coord1);
        new_tex_vertices.push_back(tex_coord2);
        new_tex_vertices.push_back(tex_coord3);

        auto new_triangle= std::make_shared<triangle>(0,material_id, texture_ids,materials, faces[i],
                                                      tex_coord_data,vertex_data, transformations,
                                                      translations, rotations, scalings, composites,vertex_offset, texture_offset,
                                                      radiance, is_light, is_cloud);

        mesh_triangles_shape.push_back(new_triangle);
        mesh_triangles.push_back(new_triangle);
        face_vertices.push_back(new_face_vertices);
        face_tex_coords.push_back(new_tex_vertices);
    }


    if(is_light){
        std::vector<float> triangle_areas;
        float total_area = 0.0;
        for(auto &mesh_tri: mesh_triangles){
            std::vector<parser::Vec3f> corners = mesh_tri->get_corner_world_coordinates();
            float area = triangle_area(corners[0], corners[1], corners[2]);

            triangle_areas.push_back(area);
            total_area += area;

        }

        total_mesh_area = total_area;
        for(int i = 0; i < triangle_areas.size(); i++){

            triangle_areas[i] = triangle_areas[i] / total_area;

        }

        float accumulation = 0.0;
        for(int i = 0; i < triangle_areas.size(); i++){

            this->cdf.push_back(triangle_areas[i] + accumulation);
            accumulation += triangle_areas[i];

        }


    }else{
        total_mesh_area = 0.0;
    }

}


parser::Vec3f mesh::get_mesh_object_light_ray(float random_cdf, float random_tri1, float random_tri2){

    //GET A TRIANGLE FROM THE CDF
    int index = -1;
    for(int i = 0; i < cdf.size(); i++){
        if(i == cdf.size() - 1){
            index = i;
        }

        if(random_cdf > cdf[i] && random_cdf < cdf[i+1]){
            index = i;
            break;
        }
    }

    float reshuffled_tri1 = sqrt(random_tri1);

    std::vector<parser::Vec3f> corners = mesh_triangles[index]->get_corner_world_coordinates();

    parser::Vec3f a_mod = vector_multiply(corners[0], 1 - reshuffled_tri1);
    parser::Vec3f b_mod = vector_multiply(corners[1], reshuffled_tri1 * (1.0f - random_tri2));
    parser::Vec3f c_mod = vector_multiply(corners[2], random_tri2 * reshuffled_tri1);

    parser::Vec3f point = vector_add(vector_add(a_mod, b_mod), c_mod);

    return point;
}



std::vector<parser::Vec3i>& mesh::get_faces(){
    return faces;
}

std::vector<std::vector<parser::Vec3f>>& mesh::get_face_vertices() {
    return face_vertices;
}

std::vector<std::shared_ptr<triangle>>& mesh::get_triangles(){
    return mesh_triangles;
}

std::vector<std::shared_ptr<shape>>& mesh::get_triangles_shape() {
    return mesh_triangles_shape;
}

Eigen::Matrix4f mesh::get_transformation_matrix(){
    return transformation_matrix;
}
