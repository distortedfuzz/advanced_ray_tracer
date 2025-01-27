#ifndef HW1_NEW_MATH_H
#define HW1_NEW_MATH_H
#include <vector>
#include <cmath>
#include <random>
#include <iostream>
#include "../parser/parser.h"
#include "../transformations/scaling.h"
#include "../transformations/translation.h"
#include "../transformations/rotation.h"
#include "../transformations/composite.h"

inline parser::Vec3f cross_product(parser::Vec3f vec1, parser::Vec3f vec2){
    parser::Vec3f product;

    product.x = (vec1.y*vec2.z - vec1.z*vec2.y);
    product.y = -(vec1.x*vec2.z - vec1.z*vec2.x);
    product.z = (vec1.x*vec2.y - vec1.y*vec2.x);

    return product;
}

inline float dot_product(parser::Vec3f vec1, parser::Vec3f vec2){
    return (vec1.x*vec2.x) + (vec1.y*vec2.y) + (vec1.z*vec2.z);
}

inline float dot_product_2(parser::Vec2f vec1, parser::Vec2f vec2){
    return (vec1.u*vec2.u) + (vec1.v*vec2.v);
}

inline parser::Vec3f vector_add(parser::Vec3f vec1, parser::Vec3f vec2) {
    parser::Vec3f vec;
    vec.x = vec1.x + vec2.x;
    vec.y = vec1.y + vec2.y;
    vec.z = vec1.z + vec2.z;

    return vec;
}

inline parser::Vec2f vector_add_2(parser::Vec2f vec1, parser::Vec2f vec2) {
    parser::Vec2f vec;
    vec.u = vec1.u + vec2.u;
    vec.v = vec1.v + vec2.v;

    return vec;
}



inline parser::Vec3f vector_subtract(parser::Vec3f vec1, parser::Vec3f vec2){
    parser::Vec3f vec;
    vec.x = vec1.x - vec2.x;
    vec.y = vec1.y - vec2.y;
    vec.z = vec1.z - vec2.z;

    return vec;
}

inline parser::Vec2f vector_subtract_2(parser::Vec2f vec1, parser::Vec2f vec2) {
    parser::Vec2f vec;
    vec.u = vec1.u - vec2.u;
    vec.v = vec1.v - vec2.v;

    return vec;
}


inline float get_vector_magnitude(parser::Vec3f vec1){

    float result;
    result = sqrt(pow(vec1.x,2) + pow(vec1.y,2) + pow(vec1.z,2));
    return result;

}

inline parser::Vec3f normalize_vector(parser::Vec3f vec){
    parser::Vec3f normalized;

    float magnitude;
    magnitude = sqrt(pow(vec.x,2) + pow(vec.y,2) + pow(vec.z,2));

    normalized.x = vec.x / magnitude;
    normalized.y = vec.y / magnitude;
    normalized.z = vec.z / magnitude;

    return normalized;
}

inline parser::Vec3f vector_multiply(parser::Vec3f vec1, float multiplier){
    parser::Vec3f vec;
    vec.x = vec1.x*multiplier;
    vec.y = vec1.y*multiplier;
    vec.z = vec1.z*multiplier;

    return vec;
}

inline parser::Vec3f vector_divide(parser::Vec3f vec1, float div){
    parser::Vec3f vec;
    vec.x = vec1.x/div;
    vec.y = vec1.y/div;
    vec.z = vec1.z/div;

    return vec;
}

inline float vector_divide_full_time(parser::Vec3f vec1, parser::Vec3f vec2){
    return vec1.x / vec2.x;
}

inline float get_3x3_determinant(parser::Vec3f vec1, parser::Vec3f vec2, parser::Vec3f vec3){

    float result;

    result = vec1.x*((vec2.y*vec3.z)-(vec3.y*vec2.z))
             - vec2.x*((vec1.y*vec3.z)-(vec3.y*vec1.z))
             + vec3.x*((vec1.y*vec2.z)-(vec2.y*vec1.z));

    return result;
}

inline float clamp(float number){
    if(number<=0){
        number = 0;
    }else if(number>255){
        number = 255;
    }
    return number;
}


inline Eigen::Matrix4f calculate_transformation_matrix(const std::vector<parser::Transformation> &transformations,
                                                 const std::vector<parser::Translation> &translations,
                                                 const std::vector<parser::Rotation> &rotations,
                                                 const std::vector<parser::Scaling> &scalings,
                                                 const std::vector<parser::Composite> &composites){

    //transformations
    if(transformations.size() > 0){
        Eigen::Matrix4f full_transformation_matrix = Eigen::Matrix4f::Identity();
        for(int i = 0; i < transformations.size(); i++){
            if(transformations[i].transformation_type == 0){
                translation new_translation(transformations[i].id - 1,
                                            translations[transformations[i].id - 1].translation_x,
                                            translations[transformations[i].id - 1].translation_y,
                                            translations[transformations[i].id - 1].translation_z);
                full_transformation_matrix = new_translation.get_translation_matrix() * full_transformation_matrix;

            } else if(transformations[i].transformation_type == 1){
                scaling new_scaling(transformations[i].id - 1,
                                    scalings[transformations[i].id - 1].scaling_x,
                                    scalings[transformations[i].id - 1].scaling_y,
                                    scalings[transformations[i].id - 1].scaling_z);
                full_transformation_matrix = new_scaling.get_scaling_matrix() * full_transformation_matrix;

            } else if(transformations[i].transformation_type == 2){
                rotation new_rotation(transformations[i].id - 1,
                                      rotations[transformations[i].id - 1].angle,
                                      rotations[transformations[i].id - 1].rotation_x,
                                      rotations[transformations[i].id - 1].rotation_y,
                                      rotations[transformations[i].id - 1].rotation_z);
                full_transformation_matrix = new_rotation.get_rotation_matrix() * full_transformation_matrix;
            }else if(transformations[i].transformation_type == 3){
                composite new_composite(transformations[i].id -1,
                                        composites[transformations[i].id -1].elements);
                full_transformation_matrix = new_composite.get_composite_matrix() * full_transformation_matrix;
            }
        }
        return full_transformation_matrix;
    } else {
        return Eigen::Matrix4f::Identity();
    }
}

inline std::vector<std::pair<float,float>> generate_samples_jittered(int number_of_samples, float cell_size, bool is_minus, std::mt19937 &random_generator){


    std::uniform_real_distribution<> random_distribution(0.0f, 1.0f);

    int count_sqrt = sqrt(number_of_samples);
    float top_start = 0.0;
    float left_start = 0.0;

    float sub_cell_size = cell_size / count_sqrt;

    std::vector<std::pair<float,float>> samples_2d;



    for(int z = 0; z < count_sqrt; z++){
        left_start = 0.0;
        for(int j = 0; j < count_sqrt; j++){

            float rand_width = random_distribution(random_generator);
            //negative as i have top left and need to go down
            float rand_height_negative = -random_distribution(random_generator);


            //std::cout<<rand_width<<" "<<rand_height_negative<<std::endl;

            float point_u = ((j * sub_cell_size) + (rand_width * sub_cell_size)) / cell_size;
            float point_v = 0.0;

            if(is_minus){
                point_v = ((-z * sub_cell_size) + (rand_height_negative * sub_cell_size)) / cell_size;
            }else{
                point_v = ((z * sub_cell_size) + (rand_height_negative * sub_cell_size)) / cell_size;
            }

            std::pair<float,float> new_sample(point_u, point_v);
            samples_2d.push_back(new_sample);

            left_start += cell_size;

        }

        top_start -= cell_size;


    }

    return samples_2d;


}


inline float get_gaussian_value(float x, float y, float sigma){

    float mult1 = 1.0f / (2.0f * M_PI * pow(sigma, 2));
    float exp_component = -0.5f * ( (pow(x,2) + pow(y,2)) / pow(sigma, 2));
    float mult2 = exp(exp_component);

    return mult1*mult2;

}



inline int hash_perlin(int x, int y, int z, int mod){
    int permutation[] = { 151, 160, 137,  91,  90,  15, 131,  13, 201,  95,  96,  53, 194, 233,   7, 225,
                          140,  36, 103,  30,  69, 142,   8,  99,  37, 240,  21,  10,  23, 190,   6, 148,
                          247, 120, 234,  75,   0,  26, 197,  62,  94, 252, 219, 203, 117,  35,  11,  32,
                          57, 177,  33,  88, 237, 149,  56,  87, 174,  20, 125, 136, 171, 168,  68, 175,
                          74, 165,  71, 134, 139,  48,  27, 166,  77, 146, 158, 231,  83, 111, 229, 122,
                          60, 211, 133, 230, 220, 105,  92,  41,  55,  46, 245,  40, 244, 102, 143,  54,
                          65,  25,  63, 161,   1, 216,  80,  73, 209,  76, 132, 187, 208,  89,  18, 169,
                          200, 196, 135, 130, 116, 188, 159,  86, 164, 100, 109, 198, 173, 186,   3,  64,
                          52, 217, 226, 250, 124, 123,   5, 202,  38, 147, 118, 126, 255,  82,  85, 212,
                          207, 206,  59, 227,  47,  16,  58,  17, 182, 189,  28,  42, 223, 183, 170, 213,
                          119, 248, 152,   2,  44, 154, 163,  70, 221, 153, 101, 155, 167,  43, 172,   9,
                          129,  22,  39, 253,  19,  98, 108, 110,  79, 113, 224, 232, 178, 185, 112, 104,
                          218, 246,  97, 228, 251,  34, 242, 193, 238, 210, 144,  12, 191, 179, 162, 241,
                          81,  51, 145, 235, 249,  14, 239, 107,  49, 192, 214,  31, 181, 199, 106, 157,
                          184,  84, 204, 176, 115, 121,  50,  45, 127,   4, 150, 254, 138, 236, 205,  93,
                          222, 114,  67,  29,  24,  72, 243, 141, 128, 195,  78,  66, 215,  61, 156, 180 };

    int hold = 3010349 * x + 2097593 * y + 3411949 * z;
    hold = (hold ^ 41);
    hold ^= (hold >> 2);

    int perm1 =  permutation[abs(hold) % 255];


    return perm1 % mod;

}

inline float weight_perlin(float x){

    float abs_x = fabs(x);

    if(abs_x < 1){
        return - (6 * pow(abs_x, 5)) + (15 * pow(abs_x, 4)) - (10 * pow(abs_x, 3)) + 1;
    }else{
        return 0.0;
    }

}


inline parser::Vec3f get_random_rejection_sample(std::mt19937 &random_generator, parser::Vec3f &normal){

    std::uniform_real_distribution<> gNURandomDistribution(-1.0f, 1.0f);
    parser::Vec3f proposed_sample;

    bool valid_sample = false;

    while(!valid_sample){
        float rand1 = gNURandomDistribution(random_generator);
        float rand2 = gNURandomDistribution(random_generator);
        float rand3 = gNURandomDistribution(random_generator);

        proposed_sample = {rand1, rand2, rand3};

        if(dot_product(proposed_sample, normal) > 0.0f){
            valid_sample = true;
        }
    }

    return normalize_vector(proposed_sample);

}


inline float triangle_area(parser::Vec3f &v1, parser::Vec3f &v2, parser::Vec3f &v3){

    parser::Vec3f a_b = vector_subtract(v2, v1);
    parser::Vec3f a_c = vector_subtract(v3, v1);

    return get_vector_magnitude(cross_product(a_b, a_c)) / 2.0f;

}

inline parser::Vec3f calculate_normal(parser::Vec3f &v1, parser::Vec3f &v2, parser::Vec3f &v3){
    parser::Vec3f a_b = normalize_vector(vector_subtract(v2, v1));
    parser::Vec3f a_c = normalize_vector(vector_subtract(v3, v1));

    return normalize_vector(cross_product(a_b, a_c));
}


inline float phase_function(float asymmetry_param, float cos_theta){

    float pi_term = 1 / (4 * M_PI);

    float upper = 1.0f - pow(asymmetry_param, 2);
    float lower = 1.0f + pow(asymmetry_param, 2) - 2*asymmetry_param* pow(cos_theta,3/2);

    return pi_term * upper / lower;

}

#endif
