#ifndef ADVANCED_RAY_TRACER_TEXTURE_H
#define ADVANCED_RAY_TRACER_TEXTURE_H

#include "../parser/parser.h"
#include "../camera/camera.h"
#include "../math/math.h"

class texture {


public:
    int shape_id;
    int image_id;
    bool is_exr;
    int width, height, channels;
    unsigned char* image_data;
    float* image_data_exr;
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
     * 0 = nearest
     * 1 = bilinear
     * 2 = trilinear (will not be implemented right now)
     */
    int interpolation;
    /*
     * 0 = absvalue
     * 1 = linear
     */
    int noise_conversion;

    float bump_factor;
    float noise_scale;
    int num_octaves;
    int normalizer;

    /*
     * 0 = image
     * 1 = perlin
     * 2 = checkerboard
     * 3 = worley
     */
    int texture_type;

    float scale;
    float offset;
    parser::Vec3f black_color;
    parser::Vec3f white_color;

    float grid_size;

    std::vector<parser::Vec3f> gradients;


    texture(int image_id, int normalizer, std::vector<parser::Image> &images, int decal_mode, int interpolation,
            int noise_conversion, float bump_factor, float noise_scale, int num_octaves, int tex_type, float scale,
            float offset, parser::Vec3f &black_color, parser::Vec3f &white_color);

    parser::Vec3f get_texture_value_procedural(parser::Vec3f &coordinate);
    parser::Vec3f get_texture_value_perlin(parser::Vec3f &coordinate);
    parser::Vec3f get_texture_value_checkerboard(parser::Vec3f &coordinate);

    parser::Vec3f get_texture_value(parser::Vec2f &coordinate);
    parser::Vec3f get_texture_value_nearest(parser::Vec2f &coordinate);
    parser::Vec3f get_texture_value_bilinear(parser::Vec2f &coordinate);

    parser::Vec3f get_texture_normal_bump(parser::Vec2f &coordinate);

    parser::Vec3f get_specific_pixel(parser::Vec2i coordinate);

};


#endif
