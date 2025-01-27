#ifndef HW1_MESH_TRIANGLE_REFRACTION_H
#define HW1_MESH_TRIANGLE_REFRACTION_H
#include <vector>
#include <cmath>
#include "../parser/parser.h"
#include "../math/math.h"

struct fresnel{
    float reflection_ratio;
    float transmission_ratio;
};


inline parser::Vec3f refraction_ray(parser::Vec3f direction, parser::Vec3f normal,float refraction_index1, float refraction_index2){

    //angle between ray and normal
    float cos_t= -dot_product(vector_multiply(direction, -1),normal)/((get_vector_magnitude(direction)* get_vector_magnitude(normal)));

    //angle between refraction ray and normal
    float cos_a = sqrt(1- ( pow(refraction_index1/refraction_index2,2) * (1-pow(fabs(cos_t),2)) ) );

    parser::Vec3f refraction_ray = vector_subtract(vector_multiply(vector_add(direction,
                                                                              vector_multiply(normal, fabs(cos_t))),
                                                                   refraction_index1 / refraction_index2),
                                                   vector_multiply(normal, cos_a));

    return refraction_ray;
}

inline float get_cos_t(parser::Vec3f direction, parser::Vec3f normal){

    //angle between ray and normal
    float cos_t= -dot_product(direction,normal)/(get_vector_magnitude(direction)* get_vector_magnitude(normal));

    return cos_t;
}

inline float get_cos_a(float cos_t, float refraction_index1 ,float refraction_index2){

    //angle between refraction ray and normal
    float cos_a = sqrt(1- ( pow(refraction_index1/refraction_index2,2) * (1-pow(fabs(cos_t),2)) ) );

    return cos_a;
}

inline float inside_cos_a(float cos_t, float refraction_index1 ,float refraction_index2){

    //angle between refraction ray and normal
    float in_cos_a = 1- ( pow(refraction_index1/refraction_index2,2) * (1-pow(fabs(cos_t),2)) );

    return in_cos_a;
}



inline fresnel get_fresnel_dielectric(float cos_t, float cos_a, float refraction_index1, float refraction_index2){


    float r1 = (refraction_index2 * fabs(cos_t) - refraction_index1 * cos_a)/ (refraction_index2 * fabs(cos_t) + refraction_index1 * cos_a);
    float r2 = (refraction_index1 * fabs(cos_t) - refraction_index2 * cos_a)/ (refraction_index1 * fabs(cos_t) + refraction_index2 * cos_a);

    fresnel new_fresnel;
    new_fresnel.reflection_ratio = (pow(r1,2)+ pow(r2,2))/2;
    if(new_fresnel.reflection_ratio> 1.0){
        new_fresnel.reflection_ratio =1.0;
    }
    if(new_fresnel.reflection_ratio< 0.0){
        new_fresnel.reflection_ratio =0.0;
    }
    new_fresnel.transmission_ratio = 1 - new_fresnel.reflection_ratio;

    return new_fresnel;
}


inline fresnel get_fresnel_conductor(float cos_t, float refraction_index, float absorption_index){

    float rs = ((pow(refraction_index,2) + pow(absorption_index,2)) - (2*refraction_index*cos_t) + pow(cos_t,2))/
                ((pow(refraction_index,2) + pow(absorption_index,2)) + (2*refraction_index*cos_t) + pow(cos_t,2));

    float rp = (((pow(refraction_index,2) + pow(absorption_index,2))* pow(cos_t,2)) - (2*refraction_index*fabs(cos_t)) + 1)/
                (((pow(refraction_index,2) + pow(absorption_index,2))* pow(cos_t,2)) + (2*refraction_index*fabs(cos_t)) + 1);



    fresnel new_fresnel;
    new_fresnel.reflection_ratio = (rs+rp)/2;
    new_fresnel.transmission_ratio = 0.0;


    return new_fresnel;
}


inline parser::Vec3f attenuation(parser::Vec3f starting_luminance, parser::Vec3f absorption_coefficient, float distance){

    parser::Vec3f end_luminance;

    end_luminance.x = starting_luminance.x * exp(-1*absorption_coefficient.x*distance);
    end_luminance.y = starting_luminance.y * exp(-1*absorption_coefficient.y*distance);
    end_luminance.z = starting_luminance.z * exp(-1*absorption_coefficient.z*distance);

    return end_luminance;

}

#endif
