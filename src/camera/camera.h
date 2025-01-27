#ifndef HW1_NEW_CAMERA_H
#define HW1_NEW_CAMERA_H
#include "../parser/parser.h"
#include "Eigen/Dense"
#include "../camera/camera.h"

struct primary_ray {

    parser::Vec3f direction;


};

struct ray {

    parser::Vec3f direction;
    parser::Vec3f start_pos;
    std::pair<float, float> near_plane_uv;
    float time;
    float pixel_cast_u;
    float pixel_cast_v;
    float aperture_u;
    float aperture_v;

};


class camera {
    private:
        int id;
        bool is_look_at;
        parser::Vec3f position;
        parser::Vec3f gaze;
        parser::Vec3f gaze_point;
        parser::Vec3f up;
        parser::Vec4f near_plane;
        parser::Vec3f top_left_point;
        float fov_y;
        float near_distance;
        float near_plane_width;
        float near_plane_height;
        int image_height;
        int image_width;
        int num_samples;
        float focus_distance;
        float aperture_size;
        std::string name;
        Eigen::Matrix4f transformation_matrix;
        std::vector<parser::Transformation> transformations;
        bool left_handed;
        bool ignore_np;
        bool path_tracing;
        /*
        * [0] == importance sampling
        * [1] == NEE
        * [2] == Russian Roulette
        */
        std::vector<bool> path_tracing_params;
        int splitting_factor;


    public:
        camera(int id,
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
               int splitting_factor);

        std::vector<ray> get_ray_directions();
        void get_jittered_ray_directions(int count, std::vector<ray> &rays);
        void get_jittered_ray_directions_dof(int count, std::vector<ray> &rays);

        parser::Vec3f get_near_plane_top_left();
        parser::ToneMap tone_map;
        std::vector<parser::Vec3f> pixel_mid_points;
        float pixel_size;
        int get_id();
        int get_num_samples();
        float get_aperture_size();
        parser::Vec3f get_position();
        parser::Vec3f get_gaze();
        parser::Vec3f get_up();
        parser::Vec4f get_near_plane();
        float get_near_distance();
        int get_image_height();
        int get_image_width();
        std::string get_name();
        bool get_ignore();
        bool is_path_tracing();
        int get_splitting_factor();
        std::vector<bool> get_path_tracing_params();
};


#endif
