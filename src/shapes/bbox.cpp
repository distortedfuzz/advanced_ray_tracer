#include "bbox.h"
#include <iostream>
#include <utility>

bbox::bbox(std::vector<std::shared_ptr<shape>> &shapes,
           int start, int end) : shape("bbox") {

    float max_x, min_x;
    float max_y, min_y;
    float max_z, min_z;

    for(int i = start; i < end; i++){
        std::vector<float> current_min_max = shapes[i]->get_min_max();

        if(i == start){
            max_x = current_min_max[0];
            min_x = current_min_max[1];

            max_y = current_min_max[2];
            min_y = current_min_max[3];

            max_z = current_min_max[4];
            min_z = current_min_max[5];
        }else{
            if(current_min_max[0] > max_x){
                max_x = current_min_max[0];
            }
            if(current_min_max[1] < min_x){
                min_x = current_min_max[1];
            }
            if(current_min_max[2] > max_y){
                max_y = current_min_max[2];
            }
            if(current_min_max[3] < min_y){
                min_y = current_min_max[3];
            }
            if(current_min_max[4] > max_z){
                max_z = current_min_max[4];
            }
            if(current_min_max[5] < min_z){
                min_z = current_min_max[5];
            }
        }
    }


    parser::Vec3f all_max{max_x, max_y, max_z};
    parser::Vec3f all_min{min_x, min_y, min_z};
    min_max_corners.push_back(all_min);
    min_max_corners.push_back(all_max);

    parser::Vec3f corner_0{min_x, min_y,min_z};
    parser::Vec3f corner_1{min_x, min_y,max_z};
    parser::Vec3f corner_2{min_x, max_y,min_z};
    parser::Vec3f corner_3{min_x, max_y,max_z};
    parser::Vec3f corner_4{max_x, min_y,min_z};
    parser::Vec3f corner_5{max_x, min_y,max_z};
    parser::Vec3f corner_6{max_x, max_y,min_z};
    parser::Vec3f corner_7{max_x, max_y,max_z};

    all_corners.push_back(corner_0);
    all_corners.push_back(corner_1);
    all_corners.push_back(corner_2);
    all_corners.push_back(corner_3);
    all_corners.push_back(corner_4);
    all_corners.push_back(corner_5);
    all_corners.push_back(corner_6);
    all_corners.push_back(corner_7);

    this->is_full_object = false;
    this->object_type = -1;
    this->object_index_in_tlas_vector = 0;
    this->transformation_matrix = Eigen::Matrix4f::Identity();
    this->motion_blur_matrix = Eigen::Matrix4f::Identity();

}

bbox::bbox(std::vector<std::shared_ptr<shape>> &shapes, int start, int end,
           bool is_full_object, int type, int index_in_tlas, parser::Material &material,
           bool is_light,bool is_cloud, parser::Vec3f &radiance,std::vector<int> &tex_ids) : shape("bbox"){

    float max_x, min_x;
    float max_y, min_y;
    float max_z, min_z;
    this->is_full_object = is_full_object;
    this->object_type = type;
    this->is_light = is_light;
    this->is_cloud = is_cloud;
    this->radiance = radiance;
    this->object_index_in_tlas_vector = index_in_tlas;
    this->bbox_material = material;
    this->transformation_matrix = Eigen::Matrix4f::Identity();

    for(auto tex: tex_ids){
        this->texture_ids.push_back(tex-1);
    }


    for(int i = start; i < end; i++){
        std::vector<float> current_min_max = shapes[i]->get_min_max();

        if(i == start){
            max_x = current_min_max[0];
            min_x = current_min_max[1];

            max_y = current_min_max[2];
            min_y = current_min_max[3];

            max_z = current_min_max[4];
            min_z = current_min_max[5];
        }else{
            if(current_min_max[0] > max_x){
                max_x = current_min_max[0];
            }
            if(current_min_max[1] < min_x){
                min_x = current_min_max[1];
            }
            if(current_min_max[2] > max_y){
                max_y = current_min_max[2];
            }
            if(current_min_max[3] < min_y){
                min_y = current_min_max[3];
            }
            if(current_min_max[4] > max_z){
                max_z = current_min_max[4];
            }
            if(current_min_max[5] < min_z){
                min_z = current_min_max[5];
            }
        }
    }


    parser::Vec3f all_max{max_x, max_y, max_z};
    parser::Vec3f all_min{min_x, min_y, min_z};

    parser::Vec3f corner_0{min_x, min_y,min_z};
    parser::Vec3f corner_1{min_x, min_y,max_z};
    parser::Vec3f corner_2{min_x, max_y,min_z};
    parser::Vec3f corner_3{min_x, max_y,max_z};
    parser::Vec3f corner_4{max_x, min_y,min_z};
    parser::Vec3f corner_5{max_x, min_y,max_z};
    parser::Vec3f corner_6{max_x, max_y,min_z};
    parser::Vec3f corner_7{max_x, max_y,max_z};

    all_corners.push_back(corner_0);
    all_corners.push_back(corner_1);
    all_corners.push_back(corner_2);
    all_corners.push_back(corner_3);
    all_corners.push_back(corner_4);
    all_corners.push_back(corner_5);
    all_corners.push_back(corner_6);
    all_corners.push_back(corner_7);

    min_max_corners.push_back(all_min);
    min_max_corners.push_back(all_max);
    this->transformation_matrix = Eigen::Matrix4f::Identity();
    this->motion_blur_matrix = Eigen::Matrix4f::Identity();
}


bbox::bbox(Eigen::Matrix4f &matrix,std::shared_ptr<bbox> &start, std::shared_ptr<bbox> &end,
           bool is_full_object, int type, int index_in_tlas, parser::Material &material,
           bool is_light, bool is_cloud, parser::Vec3f &radiance, std::vector<int> & tex_ids) : shape("bbox"){

    float max_x, min_x;
    float max_y, min_y;
    float max_z, min_z;
    this->is_full_object = is_full_object;
    this->object_type = type;
    this->object_index_in_tlas_vector = index_in_tlas;
    this->is_light = is_light;
    this->is_cloud = is_cloud;
    this->radiance = radiance;
    this->bbox_material = material;
    this->transformation_matrix = Eigen::Matrix4f::Identity();
    for(auto tex: tex_ids){
        this->texture_ids.push_back(tex-1);
    }

    for(int i = 0; i < 2; i++){
        std::vector<float> current_min_max;
        if(i == 0){
            current_min_max = start->get_min_max();
        }else{
            current_min_max = end->get_min_max();
        }


        if(i == 0){
            max_x = current_min_max[0];
            min_x = current_min_max[1];

            max_y = current_min_max[2];
            min_y = current_min_max[3];

            max_z = current_min_max[4];
            min_z = current_min_max[5];
        }else{
            if(current_min_max[0] > max_x){
                max_x = current_min_max[0];
            }
            if(current_min_max[1] < min_x){
                min_x = current_min_max[1];
            }
            if(current_min_max[2] > max_y){
                max_y = current_min_max[2];
            }
            if(current_min_max[3] < min_y){
                min_y = current_min_max[3];
            }
            if(current_min_max[4] > max_z){
                max_z = current_min_max[4];
            }
            if(current_min_max[5] < min_z){
                min_z = current_min_max[5];
            }
        }
    }


    parser::Vec3f all_max{max_x, max_y, max_z};
    parser::Vec3f all_min{min_x, min_y, min_z};

    parser::Vec3f corner_0{min_x, min_y,min_z};
    parser::Vec3f corner_1{min_x, min_y,max_z};
    parser::Vec3f corner_2{min_x, max_y,min_z};
    parser::Vec3f corner_3{min_x, max_y,max_z};
    parser::Vec3f corner_4{max_x, min_y,min_z};
    parser::Vec3f corner_5{max_x, min_y,max_z};
    parser::Vec3f corner_6{max_x, max_y,min_z};
    parser::Vec3f corner_7{max_x, max_y,max_z};

    all_corners.push_back(corner_0);
    all_corners.push_back(corner_1);
    all_corners.push_back(corner_2);
    all_corners.push_back(corner_3);
    all_corners.push_back(corner_4);
    all_corners.push_back(corner_5);
    all_corners.push_back(corner_6);
    all_corners.push_back(corner_7);

    min_max_corners.push_back(all_min);
    min_max_corners.push_back(all_max);
    this->transformation_matrix = Eigen::Matrix4f::Identity();
    this->motion_blur_matrix = matrix;
}


//can just use two opposite corners
bool bbox::does_intersect(const parser::Vec3f &cam_position, const parser::Vec3f &direction){

    float ox = cam_position.x;
    float oy = cam_position.y;
    float oz = cam_position.z;

    float dx = direction.x;
    float dy = direction.y;
    float dz = direction.z;

    float tx_min, ty_min, tz_min;
    float tx_max, ty_max, tz_max;

    float a = 1.0/dx;
    if(a>=0){
        tx_min = (min_max_corners[0].x - ox) * a;
        tx_max = (min_max_corners[1].x - ox) * a;
    }else{
        tx_min = (min_max_corners[1].x - ox) * a;
        tx_max = (min_max_corners[0].x - ox) * a;
    }

    float b = 1.0/dy;
    if(b>=0){
        ty_min = (min_max_corners[0].y - oy) * b;
        ty_max = (min_max_corners[1].y - oy) * b;
    }else{
        ty_min = (min_max_corners[1].y - oy) * b;
        ty_max = (min_max_corners[0].y - oy) * b;
    }

    float c = 1.0/dz;
    if(c>=0){
        tz_min = (min_max_corners[0].z - oz) * c;
        tz_max = (min_max_corners[1].z - oz) * c;
    }else{
        tz_min = (min_max_corners[1].z - oz) * c;
        tz_max = (min_max_corners[0].z - oz) * c;
    }


    float t0 = std::max({tx_min, ty_min, tz_min});
    float t1 = std::min({tx_max, ty_max, tz_max});

    return (t0 < (t1 + 1e-3f) && t1 > 1e-3f);
}

parser::Vec3f bbox::get_center(){

    return parser::Vec3f{static_cast<float>((min_max_corners[0].x + min_max_corners[1].x)/2.0),
                         static_cast<float>((min_max_corners[0].y + min_max_corners[1].y)/2.0),
                         static_cast<float>((min_max_corners[0].z + min_max_corners[1].z)/2.0)};

}

parser::Vec3f bbox::get_dimension_differences(){
    return parser::Vec3f{fabs(min_max_corners[0].x - min_max_corners[1].x),
                         fabs(min_max_corners[0].y - min_max_corners[1].y),
                         fabs(min_max_corners[0].z - min_max_corners[1].z)};
}

std::vector<parser::Vec3f> bbox::get_corners(){
    return this->min_max_corners;
}


std::vector<float> bbox::get_min_max(){
    std::vector<float> min_max_points;

    min_max_points.push_back(min_max_corners[1].x+0.000001);
    min_max_points.push_back(min_max_corners[0].x-0.000001);
    min_max_points.push_back(min_max_corners[1].y+0.000001);
    min_max_points.push_back(min_max_corners[0].y-0.000001);
    min_max_points.push_back(min_max_corners[1].z+0.000001);
    min_max_points.push_back(min_max_corners[0].z-0.000001);
    return min_max_points;
}


void bbox::apply_transformation(Eigen::Matrix4f transformation_matrix){


    this->transformation_matrix = transformation_matrix*this->transformation_matrix;

    //transform all vectors
    for(int i = 0; i<8;i++){
        Eigen::Vector4f vec;
        vec << all_corners[i].x, all_corners[i].y, all_corners[i].z, 1.0;
        vec = transformation_matrix*vec;
        if(vec(3) != 1.0){
            vec = vec/vec(3);
        }

        all_corners[i].x = vec(0);
        all_corners[i].y = vec(1);
        all_corners[i].z = vec(2);
    }

    //get new min-max
    float max_x, min_x;
    float max_y, min_y;
    float max_z, min_z;
    /*std::cout<<"all corners before transformation:"<<std::endl;
    for(int i = 0; i<8; i++){
        std::cout<<all_corners[i].x<<" "<<all_corners[i].y<<" "<<all_corners[i].z<<std::endl;
    }*/

    for(int i = 0; i < 8; i++){
        if(i == 0){
            max_x = all_corners[i].x;
            min_x = all_corners[i].x;

            max_y = all_corners[i].y;
            min_y = all_corners[i].y;

            max_z = all_corners[i].z;
            min_z = all_corners[i].z;
        }else{
            if(all_corners[i].x > max_x){
                max_x = all_corners[i].x;
            }
            if(all_corners[i].x < min_x){
                min_x = all_corners[i].x;
            }
            if(all_corners[i].y > max_y){
                max_y = all_corners[i].y;
            }
            if(all_corners[i].y < min_y){
                min_y = all_corners[i].y;
            }
            if(all_corners[i].z > max_z){
                max_z = all_corners[i].z;
            }
            if(all_corners[i].z < min_z){
                min_z = all_corners[i].z;
            }
        }
    }


    min_max_corners[0].x = min_x;
    min_max_corners[0].y = min_y;
    min_max_corners[0].z = min_z;

    min_max_corners[1].x = max_x;
    min_max_corners[1].y = max_y;
    min_max_corners[1].z = max_z;


    all_corners = {
            {min_x, min_y, min_z},
            {min_x, min_y, max_z},
            {min_x, max_y, min_z},
            {min_x, max_y, max_z},
            {max_x, min_y, min_z},
            {max_x, min_y, max_z},
            {max_x, max_y, min_z},
            {max_x, max_y, max_z}
    };

}

bool bbox::get_is_light(){
    return is_light;
}

bool bbox::get_is_cloud(){
    return is_cloud;
}

parser::Vec3f bbox::get_radiance(){
    return radiance;
}


Eigen::Matrix4f bbox::get_transformation_matrix(){
    return transformation_matrix;
}
Eigen::Matrix4f bbox::get_blur_matrix(){
    return motion_blur_matrix;
}
int bbox::get_shape_inside_shape_type(){
    return object_type;
}

int bbox::get_index_in_tlas(){
    return object_index_in_tlas_vector;
}

parser::Material bbox::get_material_struct(){

    return bbox_material;

}

std::vector<int> bbox::get_tex_ids(){

    return this->texture_ids;
}

