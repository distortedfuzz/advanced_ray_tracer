#ifndef ADVANCED_RAY_TRACER_TONEMAPPING_H
#define ADVANCED_RAY_TRACER_TONEMAPPING_H

#include "../parser/parser.h"
#include "../math/math.h"

inline float get_luminance(parser::Vec3f &rgb){

    return (rgb.x * 0.2126f + rgb.y * 0.7152f + rgb.z * 0.0722f);

}

inline float get_average_luminance(std::vector<float> &luminance_values, int number_of_pixels){

    float total_sum = 0.0f;
    for (int i = 0; i < number_of_pixels; i++) {
        total_sum += log(1e-6 + luminance_values[i]);
    }
    float average = total_sum / number_of_pixels;
    return exp(average);

}

//check for fault
inline float get_l_white(std::vector<float> &luminance_values, float burnout_percentage){

    std::vector<float> copy_vec(luminance_values);

    std::sort(copy_vec.begin(), copy_vec.end());


    float burnout_percentage_act = burnout_percentage / 100.f;
    int index = ((float)copy_vec.size())*(1.f - burnout_percentage_act);

    float l_white = copy_vec[index];

    return l_white;

}

inline float get_scaled_luminance(float luminance, float average_luminance, float key){


    return (key / average_luminance) * luminance;

}

inline float tone_map_zero_burnout(float scaled_luminance){

    return scaled_luminance / (1.0f + scaled_luminance);

}


//check for fault
inline float tone_map(float scaled_luminance, float l_white){

    return (scaled_luminance * (1.0f + (scaled_luminance / (l_white * l_white) ) ) ) /
            (1.0f + scaled_luminance);

}

inline parser::Vec3f saturation(parser::Vec3f &rgb, float yi, float yo, float saturation){

    if(yi <= 0){
        yi = 1e-10;
    }
    return parser::Vec3f{yo * pow(rgb.x / yi, saturation),
                         yo * pow(rgb.y / yi, saturation),
                         yo * pow(rgb.z / yi, saturation)};

}


inline parser::Vec3f gamma_correction(parser::Vec3f &rgb, float gamma){

    float g_rev = 1.0f / gamma;

    return parser::Vec3f{255 * pow(rgb.x, g_rev),
                         255 * pow(rgb.y, g_rev),
                         255 * pow(rgb.z, g_rev)};

}


#endif
