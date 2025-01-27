#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>

#include <zlib.h>
#include "../parser/tinyexr.h"

namespace parser
{
    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    struct Vec2i
    {
        int u, v;
    };

    struct Vec2f
    {
        float u, v;
    };

    struct Vec3f
    {
        float x, y, z;
    };

    struct Vec3i
    {
        int x, y, z;
    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Transformation{

        //0: translation
        //1: scaling
        //2: rotation
        int transformation_type;

        int id;

    };

    struct Rotation
    {
        int id;
        float angle;
        float rotation_x;
        float rotation_y;
        float rotation_z;
    };

    struct Translation
    {
        int id;
        float translation_x;
        float translation_y;
        float translation_z;
    };

    struct Scaling
    {
        int id;
        float scaling_x;
        float scaling_y;
        float scaling_z;
    };

    struct Composite
    {
        int id;
        std::vector<float> elements;
    };

    struct ToneMap
    {
        /*
         * 0: Photographic
         */
        int tmo;
        float key_value;
        float burn_percent;
        float saturation;
        float gamma;
    };

    struct Camera
    {
        bool is_look_at;
        Vec3f position;
        Vec3f gaze;
        Vec3f gaze_point;
        Vec3f up;
        Vec4f near_plane;
        float fov_y;
        float near_distance;
        int num_samples;
        int image_width, image_height;
        std::string image_name;
        std::vector<Transformation> transformations;
        float focus_distance;
        float aperture_size;
        ToneMap tone_map;
        bool handedness;
        bool ignore_np;
        bool path_tracing;
        /*
         * [0] == importance sampling
         * [1] == NEE
         * [2] == Russian Roulette
         */
        std::vector<bool> path_tracing_params;
        int splitting_factor;
    };

    struct BRDF
    {
        /*
         * 0 : Original Blinn-Phong
         * 1 : Modified Blinn-Phong
         * 2 : Original Phong
         * 3 : Modified Phong
         * 4 : Torrance Sparrow
         */
        int mode;
        int id;
        bool normalized;
        bool kdfresnel;
        float exponent;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
        std::vector<Transformation> transformations;
    };

    struct AreaLight
    {
        Vec3f position;
        Vec3f normal;
        Vec3f radiance;
        float size;
    };

    struct DirectionalLight
    {
        Vec3f direction;
        Vec3f radiance;

    };

    struct SpotLight
    {
        Vec3f position;
        Vec3f direction;
        Vec3f intensity;
        float coverage_angle;
        float falloff_angle;

    };

    struct Material
    {
        bool is_mirror;
        bool is_dielectric;
        bool is_conductor;
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        Vec3f absorption_coefficient;
        float refraction_index;
        float absorption_index;
        float phong_exponent;
        float roughness;
        int BRDF_id;
        bool degamma;
    };

    struct SphericalDirectionalLight
    {
        int id;
        int type;
        int image_id;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
    };


    struct Mesh
    {
        int id;
        int material_id;
        std::vector<Face> faces;
        std::vector<Transformation> transformations;
        Vec3f motion_blur;
        std::vector<int> texture_ids;
        int vertex_offset;
        int texture_offset;
        bool is_light;
        bool is_cloud;
        parser::Vec3f radiance;
    };


    struct MeshInstance
    {
        int id;
        int material_id;
        bool is_light;
        parser::Vec3f radiance;
        int corresponding_mesh_id;
        bool reset_transform;
        bool is_new_material;
        std::vector<Transformation> transformations;
        Vec3f motion_blur;
        std::vector<int> texture_ids;
    };


    struct Triangle
    {
        int material_id;
        Face indices;
        std::vector<Transformation> transformations;
        Vec3f motion_blur;
        std::vector<int> texture_ids;
        bool is_light;
        parser::Vec3f radiance;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
        std::vector<Transformation> transformations;
        Vec3f motion_blur;
        std::vector<int> texture_ids;
        bool is_cloud;
        bool is_light;
        parser::Vec3f radiance;
    };

    struct Image{
        int id;
        bool is_exr;
        const char* image_route;
        float* exr_data;
        unsigned char* data;
        int width, height, channels;

    };

    struct Texture{
        int id;
        int image_id;
        /*
         * decal_mode:
         * 0 = replace_kd
         * 1 = blend_kd
         * 2 = replace_ks
         * 3 = replace_background
         * 4 = replace_normal
         * 5 = bump_normal
         * 6 = replace_all
         */
        int decal_mode;
        /*
         * interpolation:
         * 0 = nearest
         * 1 = bilinear
         * 2 = trilinear (will not be implemented right now)
         */
        int interpolation;


        /*
        * 0 = image
        * 1 = perlin
        * 2 = checkerboard
        * 3 = worley
        */
        int texture_type;

        /*
        * type:
        * 0 = absvalue
        * 1 = linear
        */
        int noise_conversion;
        int num_octaves;
        float bump_factor;
        float noise_scale;
        int normalizer;

        float scale;
        float offset;
        parser::Vec3f black_color;
        parser::Vec3f white_color;

        float grid_size;

    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        /*
         * Absorption
         * Scattering
         * Asymmetry Parameter
         */
        std::vector<float> cloud_parameters;
        float shadow_ray_epsilon;
        float intersection_test_epsilon;
        int max_recursion_depth;
        int min_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<AreaLight> area_lights;
        std::vector<DirectionalLight> directional_lights;
        std::vector<SpotLight> spot_lights;
        std::vector<SphericalDirectionalLight> environment_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Vec2f> tex_coord_data;
        std::vector<Mesh> meshes;
        std::vector<MeshInstance> mesh_instances;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;
        std::vector<Translation> translations;
        std::vector<Scaling> scalings;
        std::vector<Rotation> rotations;
        std::vector<Composite> composites;
        std::vector<Image> images;
        std::vector<Texture> textures;
        std::vector<BRDF> brdfs;

        //Functions
        void loadFromXml(const std::string &filepath);
        void oldLoadFromXml(const std::string &filepath);
    };
}

#endif
