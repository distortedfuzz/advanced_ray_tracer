#include "camera.h"
#include "../math/math.h"
#include <random>
#include <iostream>
#include <algorithm>
#include <atomic>
#include <thread>

camera::camera(int id,
               bool is_look_at,
               parser::Vec3f position,
               parser::Vec3f gaze,
               parser::Vec3f gaze_point,
               parser::Vec3f up,
               parser::Vec4f near_plane,
               float fov_y,
               float near_distance,
               int image_height,
               int image_width,
               int num_samples,
               std::string name,
               float focus_distance,
               float aperture_size,
               const std::vector<parser::Transformation> &transformations,
               const std::vector<parser::Translation> &translations,
               const std::vector<parser::Rotation> &rotations,
               const std::vector<parser::Scaling> &scalings,
               const std::vector<parser::Composite> &composites,
               parser::ToneMap tone_map,
               bool left_handed,
               bool ignore_np,
               bool path_tracing,
               std::vector<bool> &path_tracing_params,
               int splitting_factor) {

    this->id = id;
    this->is_look_at = is_look_at;
    this->position = position;
    this->gaze = normalize_vector(gaze);
    this-> gaze_point = gaze_point;
    this->up = normalize_vector(up);
    this->near_plane = near_plane;
    this->fov_y = fov_y;
    this->near_distance = near_distance;
    this->image_height = image_height;
    this->image_width = image_width;
    this->num_samples = num_samples;
    this->focus_distance = focus_distance;
    this->aperture_size = aperture_size;
    this->name = name;
    this->tone_map = tone_map;
    this->left_handed = left_handed;
    this->ignore_np = ignore_np;
    this->path_tracing_params = path_tracing_params;
    this->path_tracing = path_tracing;
    this->splitting_factor = splitting_factor;

    if(this->path_tracing){
        std::cout<<"PATH TRACING"<<std::endl;
    }
    std::cout<<"params size: "<<path_tracing_params.size()<<std::endl;
    if(this->path_tracing_params[0]){
        std::cout<<"IMPORTANCE SAMPLING"<<std::endl;
    }else{
        std::cout<<"UNIFORM SAMPLING"<<std::endl;
    }

    if(this->path_tracing_params[1]){
        std::cout<<"NEXT EVENT ESTIMATION"<<std::endl;
    }

    if(this->path_tracing_params[2]){
        std::cout<<"RUSSIAN ROULETTE"<<std::endl;
    }

    if(is_look_at){
        if(gaze_point.x < INFINITY && gaze_point.y < INFINITY && gaze_point.z < INFINITY){
            this->gaze = normalize_vector(vector_subtract(gaze_point, position));
        }
    }


    this->transformation_matrix = calculate_transformation_matrix(transformations, translations, rotations, scalings, composites);

    Eigen::Vector4f gaze_hold;
    gaze_hold << this->gaze.x, this->gaze.y,this->gaze.z, 0.0;
    gaze_hold = transformation_matrix*gaze_hold;
    parser::Vec3f gaze_tr{gaze_hold(0), gaze_hold(1), gaze_hold(2)};
    this->gaze = normalize_vector(gaze_tr);


    Eigen::Vector4f up_hold;
    up_hold << this->up.x, this->up.y,this->up.z, 0.0;
    up_hold = transformation_matrix*up_hold;
    parser::Vec3f up_tr{up_hold(0), up_hold(1), up_hold(2)};
    this->up = normalize_vector(up_tr);


    Eigen::Vector4f position_hold;
    position_hold << this->position.x, this->position.y,this->position.z, 1.0;
    position_hold = transformation_matrix*position_hold;

    if(position_hold(3) != 1.0){
        position_hold = position_hold/position_hold(3);
    }

    parser::Vec3f pos_tr{position_hold(0), position_hold(1), position_hold(2)};
    this->position = pos_tr;

    if(!is_look_at){
        this->near_plane_width = fabs(near_plane.x - near_plane.y);
        this->near_plane_height = fabs(near_plane.z - near_plane.w);
    }else{
        this->near_plane_height = 2.0f * near_distance * tan((fov_y * (M_PI/180)) / 2.0f);
        this->near_plane_width = this->near_plane_height * (image_width / image_height);
    }


}

std::vector<ray> camera::get_ray_directions() {

    parser::Vec3f w_neg_vector = normalize_vector(gaze);
    parser::Vec3f u_vector = normalize_vector(cross_product(w_neg_vector, up));
    parser::Vec3f v_vector = cross_product(u_vector, w_neg_vector);



    float fov_y_radians = fov_y * (M_PI / 180.0f);
    float height = 0;
    float width = 0;

    float aspect_ratio = static_cast<float>(image_width)/static_cast<float>(image_height);

    if(is_look_at){
        height = 2 * tan(fov_y_radians / 2.0f) * near_distance;
        width = height * aspect_ratio;
    }else{
        height = near_plane.w - near_plane.z;
        width = near_plane.y - near_plane.x;
    }

    float pixel_height = height/image_height;
    float pixel_width = width/image_width;

    parser::Vec3f center_point = vector_add(position, vector_multiply(w_neg_vector, near_distance));

    if(!is_look_at){
        float diff_height = 0;
        int height_side = -1;
        if(near_plane.w != -near_plane.z){
            diff_height = fabs(fabs(near_plane.w)- fabs(near_plane.z));
            if(fabs(near_plane.w) > fabs(near_plane.z)){
                height_side = 0;
            }else{
                height_side = 1;
            }

        }

        if(height_side != -1){
            if(height_side == 0){
                center_point = vector_add(center_point, vector_multiply(v_vector, diff_height/2));
            }else{
                center_point = vector_add(center_point, vector_multiply(v_vector, -diff_height/2));
            }
        }

        float diff_width = 0;
        int width_side = -1;
        if(near_plane.y != -near_plane.x){
            diff_width = fabs(fabs(near_plane.x)- fabs(near_plane.y));
            if(fabs(near_plane.x) > fabs(near_plane.y)){
                width_side = 0;
            }else{
                width_side = 1;
            }

        }

        if(width_side != -1){
            if(width_side == 0){
                center_point = vector_add(center_point, vector_multiply(u_vector, -diff_width/2));
            }else{
                center_point = vector_add(center_point, vector_multiply(u_vector, diff_width/2));
            }
        }
    }



    parser::Vec3f top_left_point = vector_add(center_point, vector_add(vector_multiply(u_vector, -width / 2.0f),
                                                                       vector_multiply(v_vector, height / 2.0f)));


    float coordinate_u;
    float coordinate_v;
    std::vector<ray> rays;
    for(int y = 0; y<image_height; y++){

        for(int x = 0; x<image_width; x++){
            coordinate_u = ((x+0.5)*pixel_width);
            coordinate_v = (-(y+0.5)*pixel_height);

            parser::Vec3f point_s = vector_add(vector_add(top_left_point, vector_multiply(
                                                       u_vector, coordinate_u)), vector_multiply(v_vector, coordinate_v));


            parser::Vec3f new_ray_direction = vector_divide(vector_subtract(point_s, position),
                                get_vector_magnitude(vector_subtract(point_s, position)));

            ray new_ray{};

            new_ray.near_plane_uv.first = (point_s.x - top_left_point.x) / near_plane_width;
            new_ray.near_plane_uv.second = (top_left_point.y - point_s.y) / near_plane_height;

            new_ray.direction = new_ray_direction;
            new_ray.pixel_cast_u = 0;
            new_ray.pixel_cast_v = 0;
            new_ray.time = 0.0;
            new_ray.start_pos = position;
            rays.push_back(new_ray);

        }
    }

    return rays;
}


void camera::get_jittered_ray_directions(int count, std::vector<ray> &rays){

// Right-handed system (standard)
    parser::Vec3f w_neg_vector = normalize_vector(gaze);
    parser::Vec3f u_vector = normalize_vector(cross_product(w_neg_vector, up));
    parser::Vec3f v_vector = cross_product(u_vector, w_neg_vector);

// For a left-handed system:
    if (left_handed) {
        // Recalculate the cross product to ensure correct handedness
        u_vector = normalize_vector(cross_product(vector_multiply(w_neg_vector,-1), up)); // reverse the order of cross product
        v_vector = cross_product(u_vector, vector_multiply(w_neg_vector,-1)); // flip v_vector direction for left-handedness
    }

    float fov_y_radians = fov_y * (M_PI / 180.0f);
    float height = 0;
    float width = 0;

    float aspect_ratio = static_cast<float>(image_width)/static_cast<float>(image_height);

    if(is_look_at){
        height = 2 * tan(fov_y_radians / 2.0f) * near_distance;
        width = height * aspect_ratio;
    }else{
        height = near_plane.w - near_plane.z;
        width = near_plane.y - near_plane.x;
    }

    float pixel_height = height/image_height;
    float pixel_width = width/image_width;

    parser::Vec3f center_point = vector_add(position, vector_multiply(w_neg_vector, near_distance));
    if(!is_look_at){
        float diff_height = 0;
        int height_side = -1;
        if(near_plane.w != -near_plane.z){
            diff_height = fabs(fabs(near_plane.w)- fabs(near_plane.z));
            if(fabs(near_plane.w) > fabs(near_plane.z)){
                height_side = 0;
            }else{
                height_side = 1;
            }

        }

        if(height_side != -1){
            if(height_side == 0){
                center_point = vector_add(center_point, vector_multiply(v_vector, diff_height/2));
            }else{
                center_point = vector_add(center_point, vector_multiply(v_vector, -diff_height/2));
            }
        }

        float diff_width = 0;
        int width_side = -1;
        if(near_plane.y != -near_plane.x){
            diff_width = fabs(fabs(near_plane.x)- fabs(near_plane.y));
            if(fabs(near_plane.x) > fabs(near_plane.y)){
                width_side = 0;
            }else{
                width_side = 1;
            }

        }

        if(width_side != -1){
            if(width_side == 0){
                center_point = vector_add(center_point, vector_multiply(u_vector, -diff_width/2));
            }else{
                center_point = vector_add(center_point, vector_multiply(u_vector, diff_width/2));
            }
        }
    }



    parser::Vec3f top_left_point = vector_add(center_point, vector_add(vector_multiply(u_vector, -width / 2.0f),
                                                                       vector_multiply(v_vector, height / 2.0f)));

    this->pixel_size = pixel_height;


    std::uniform_real_distribution<> gNURandomDistribution(0.0f, 1.0f);
    uint32_t processing_unit_count = std::thread::hardware_concurrency();
    if (processing_unit_count == 0) {
        processing_unit_count = 8;
    }

    std::vector<std::thread> processing_units;
    processing_units.reserve(processing_unit_count);

    std::atomic<uint32_t> cursor = 0;

    for (uint32_t i = 0; i < processing_unit_count; i++) {
        processing_units.push_back(std::thread([&]() {
            std::random_device rd;
            static thread_local std::mt19937 random_generator(rd() + i);

            while (true) {
                int j = cursor.fetch_add(1, std::memory_order_relaxed);
                if (j >= this->image_height) {
                    break;
                }


                for (int i = 0; i < this->image_width; i++) {
                    parser::Vec3f pixel_top_left = vector_add(vector_add(top_left_point,
                                                                         vector_multiply(u_vector, i * pixel_width)),
                                                              vector_multiply(v_vector, -j * pixel_height));


                    std::vector<std::pair<float, float>> samples_pixel = generate_samples_jittered(count, pixel_height,
                                                                                                   true, random_generator);

                    for (int k = 0; k < count; k++) {
                        float primary_time = gNURandomDistribution(random_generator);


                        parser::Vec3f point_s = vector_add(vector_add(pixel_top_left,
                                                                      vector_multiply(u_vector, samples_pixel[k].first *
                                                                                                pixel_height)),
                                                           vector_multiply(v_vector,
                                                                           samples_pixel[k].second * pixel_height));

                        parser::Vec3f new_ray_direction = vector_divide(vector_subtract(point_s, position),
                                                                        get_vector_magnitude(
                                                                                vector_subtract(point_s, position)));

                        ray new_ray{};

                        new_ray.near_plane_uv.first = (point_s.x - top_left_point.x) / near_plane_width;
                        new_ray.near_plane_uv.second = (top_left_point.y - point_s.y) / near_plane_height;

                        new_ray.direction = new_ray_direction;
                        new_ray.pixel_cast_u = samples_pixel[k].first;
                        new_ray.pixel_cast_v = -samples_pixel[k].second;
                        new_ray.time = primary_time;
                        new_ray.start_pos = position;
                        rays[(((j * this->image_width) + i)*count) + k] = new_ray;
                    }
                }
            }
        }));
    }

    for (auto &processing_unit : processing_units) {
        processing_unit.join();
    }


    processing_units.clear();



}


void camera::get_jittered_ray_directions_dof(int count, std::vector<ray> &rays){

    parser::Vec3f w_neg_vector = normalize_vector(gaze);
    parser::Vec3f u_vector = normalize_vector(cross_product(w_neg_vector, up));
    parser::Vec3f v_vector = cross_product(u_vector, w_neg_vector);

    float fov_y_radians = fov_y * (M_PI / 180.0f);
    float height = 0;
    float width = 0;

    float aspect_ratio = static_cast<float>(image_width)/static_cast<float>(image_height);

    if(is_look_at){
        height = 2 * tan(fov_y_radians / 2.0f) * near_distance;
        width = height * aspect_ratio;
    }else{
        height = near_plane.w - near_plane.z;
        width = near_plane.y - near_plane.x;
    }

    float pixel_height = height/image_height;
    float pixel_width = width/image_width;

    parser::Vec3f center_point = vector_add(position, vector_multiply(w_neg_vector, near_distance));

    if(!is_look_at){
        float diff_height = 0;
        int height_side = -1;
        if(near_plane.w != -near_plane.z){
            diff_height = fabs(fabs(near_plane.w)- fabs(near_plane.z));
            if(fabs(near_plane.w) > fabs(near_plane.z)){
                height_side = 0;
            }else{
                height_side = 1;
            }

        }

        if(height_side != -1){
            if(height_side == 0){
                center_point = vector_add(center_point, vector_multiply(v_vector, diff_height/2));
            }else{
                center_point = vector_add(center_point, vector_multiply(v_vector, -diff_height/2));
            }
        }

        float diff_width = 0;
        int width_side = -1;
        if(near_plane.y != -near_plane.x){
            diff_width = fabs(fabs(near_plane.x)- fabs(near_plane.y));
            if(fabs(near_plane.x) > fabs(near_plane.y)){
                width_side = 0;
            }else{
                width_side = 1;
            }

        }

        if(width_side != -1){
            if(width_side == 0){
                center_point = vector_add(center_point, vector_multiply(u_vector, -diff_width/2));
            }else{
                center_point = vector_add(center_point, vector_multiply(u_vector, diff_width/2));
            }
        }
    }


    parser::Vec3f top_left_point = vector_add(center_point, vector_add(vector_multiply(u_vector, -width / 2.0f),
                                                                       vector_multiply(v_vector, height / 2.0f)));

    this->top_left_point = top_left_point;

    this->pixel_size = pixel_height;
    std::random_device rd;

    std::uniform_real_distribution<> random_distribution(0.0f, 1.0f);

    uint32_t processing_unit_count = std::thread::hardware_concurrency();
    if (processing_unit_count == 0) {
        processing_unit_count = 8;
    }

    std::vector<std::thread> processing_units;
    processing_units.reserve(processing_unit_count);

    std::atomic<uint32_t> cursor = 0;

    for (uint32_t i = 0; i < processing_unit_count; i++) {
        processing_units.push_back(std::thread([&]() {
            std::random_device rd;
            static thread_local std::mt19937 random_generator(rd() + i);

            while (true) {
                int j = cursor.fetch_add(1, std::memory_order_relaxed);
                if (j >= this->image_height) {
                    break;
                }


                for (int i = 0; i < this->image_width; i++) {
                    parser::Vec3f pixel_top_left = vector_add(vector_add(top_left_point,
                                                                         vector_multiply(u_vector, i*pixel_width)),
                                                              vector_multiply(v_vector, -j*pixel_height));



                    std::vector<std::pair<float,float>> samples_pixel = generate_samples_jittered(count, pixel_height, true,random_generator);
                    std::vector<std::pair<float,float>> samples_aperture = generate_samples_jittered(count, aperture_size, false,random_generator);

                    std::shuffle(samples_aperture.begin(), samples_aperture.end(), random_generator);


                    for (int k = 0; k < count; k++) {
                        float primary_time = random_distribution(random_generator);

                        //std::cout<<"samples: "<<samples_pixel[z].first<<" "<<samples_pixel[z].second<<std::endl;
                        parser::Vec3f point_s = vector_add(vector_add(pixel_top_left,
                                                                      vector_multiply(u_vector, samples_pixel[k].first*pixel_height)),
                                                           vector_multiply(v_vector, samples_pixel[k].second*pixel_height));

                        parser::Vec3f new_ray_direction = vector_divide(vector_subtract(point_s, position),
                                                                        get_vector_magnitude(vector_subtract(point_s, position)));

                        ray new_ray{};

                        new_ray.near_plane_uv.first = (point_s.x - top_left_point.x) / near_plane_width;
                        new_ray.near_plane_uv.second = (top_left_point.y - point_s.y) / near_plane_height;

                        new_ray.pixel_cast_u = samples_pixel[k].first;
                        new_ray.pixel_cast_v = -samples_pixel[k].second;
                        new_ray.time = primary_time;



                        float aperture_t = focus_distance/dot_product(new_ray_direction, w_neg_vector);

                        parser::Vec3f point_p = vector_add(position, vector_multiply(new_ray_direction,aperture_t));

                        //std::cout<<rand_aperture_width *aperture_size<<" "<<rand_aperture_height *aperture_size<<std::endl;

                        parser::Vec3f dof_ray_origin = vector_add(position,
                                                                  vector_add(vector_multiply(u_vector, (samples_aperture[k].first-0.5) *aperture_size),
                                                                             vector_multiply(v_vector, (samples_aperture[k].second-0.5) * aperture_size)));

                        parser::Vec3f dof_ray_direction = normalize_vector(vector_subtract(point_p, dof_ray_origin));
                        new_ray.start_pos = dof_ray_origin;
                        new_ray.direction = dof_ray_direction;

                        rays[(((j * this->image_width) + i)*count) + k] = new_ray;

                    }

                }
            }
        }));
    }


    for (auto &processing_unit : processing_units) {
        processing_unit.join();
    }


    processing_units.clear();

}


/*
std::vector<ray> camera::get_nrooks_ray_directions(int count){

    parser::Vec3f w_neg_vector = normalize_vector(gaze);
    parser::Vec3f u_vector = normalize_vector(cross_product(w_neg_vector, up));
    parser::Vec3f v_vector = cross_product(u_vector, w_neg_vector);

    float fov_y_radians = fov_y * (M_PI / 180.0f);
    float height = 0;
    float width = 0;

    float aspect_ratio = static_cast<float>(image_width)/static_cast<float>(image_height);

    if(is_look_at){
        height = 2 * tan(fov_y_radians / 2.0f) * near_distance;
        width = height * aspect_ratio;
    }else{
        height = near_plane.w - near_plane.z;
        width = near_plane.y - near_plane.x;
    }

    float pixel_height = height/image_height;
    float pixel_width = width/image_width;

    parser::Vec3f center_point = vector_add(position, vector_multiply(w_neg_vector, near_distance));
    parser::Vec3f top_left_point = vector_add(center_point, vector_add(vector_multiply(u_vector, -width / 2.0f),
                                                                       vector_multiply(v_vector, height / 2.0f)));

    float pixel_sub_width = pixel_width / count;
    float pixel_sub_height = pixel_height / count;

    float top_left_u;
    float top_left_v;
    std::vector<parser::Vec3f> rays;


    for(int y = 0; y<image_height; y++){

        for(int x = 0; x<image_width; x++){
            //i do not want middle, i want top left
            top_left_u = x*pixel_width;
            top_left_v = -y*pixel_height;

            std::default_random_engine engine;
            engine.seed(std::random_device{}());
            std::uniform_real_distribution<float> width_gen(0.0f, pixel_sub_width);
            std::uniform_real_distribution<float> height_gen(-pixel_sub_height, 0.0f);
            std::uniform_int_distribution<int> column_gen(0, count-1);

            std::vector<int> columns;
            for(int i = 0; i<count; i++){
                columns.push_back(i);
            }

            float sub_top_line = top_left_v;
            float sub_left_line = top_left_u;

            int column_picker = count-1;
            for(int z = 0; z < count; z++){

                float rand_width = width_gen(engine);
                //negative as i have top left and need to go down
                float rand_height_negative = height_gen(engine);

                float direction_point_u = sub_left_line + rand_width;
                float direction_point_v = sub_top_line + rand_height_negative;

                int column_index = column_gen(engine);
                int column = columns[column_index];
                columns.erase(columns.begin() + column_index);

                column_picker--;
                column_gen = std::uniform_int_distribution<int>(0, column_picker);

                direction_point_u += column*pixel_sub_width;

                parser::Vec3f point_s = vector_add(vector_add(top_left_point,vector_multiply(
                        u_vector,direction_point_u)),vector_multiply(v_vector,direction_point_v));

                parser::Vec3f ray = vector_divide(vector_subtract(point_s, position),
                                                  get_vector_magnitude(vector_subtract(point_s, position)));

                rays.push_back(ray);


                sub_top_line -= pixel_sub_height;
            }

        }
    }

    return rays;

}
*/


int camera::get_id(){
    return this->id;
}

parser::Vec3f camera::get_position(){
    return this->position;
}

parser::Vec3f camera::get_gaze(){
    return this->gaze;
}

parser::Vec3f camera::get_up(){
    return this->up;
}

parser::Vec4f camera::get_near_plane(){
    return this->near_plane;
}

float camera::get_aperture_size() {
    return this->aperture_size;
}

float camera::get_near_distance(){
    return this->near_distance;
}

int camera::get_num_samples() {
    return this->num_samples;
}

int camera::get_image_height(){
    return this->image_height;
}

int camera::get_image_width(){
    return this->image_width;
}

std::string camera::get_name(){
    return this->name;
}

bool camera::get_ignore(){
    return this->ignore_np;
}

bool camera::is_path_tracing(){
    return path_tracing;
}

int camera::get_splitting_factor(){
    return splitting_factor;
}

std::vector<bool> camera::get_path_tracing_params(){
    return path_tracing_params;
}