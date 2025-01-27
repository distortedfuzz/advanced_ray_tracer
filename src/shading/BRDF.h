#ifndef ADVANCED_RAY_TRACER_BRDF_H
#define ADVANCED_RAY_TRACER_BRDF_H

#include "../parser/parser.h"
#include "../math/math.h"

class BRDF {
private:
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
    float exponent;
    bool kd_fresnel;

public:
    BRDF(int mode, int id, bool normalized, float exponent, bool kd_fresnel);

    int get_id();
    int get_mode();
    bool is_normalized();
    float get_exponent();

    //MAIN HANDLER
    parser::Vec3f get_all_result(parser::Vec3f &kd,parser::Vec3f &ks, parser::Vec3f &normal, parser::Vec3f &light_direction,
                             parser::Vec3f &cam_position, parser::Vec3f &point, float refraction_index);

    //PHONG
    parser::Vec3f get_ks_phong(parser::Vec3f &ks, parser::Vec3f &normal, parser::Vec3f &light_direction,
                               parser::Vec3f &cam_position, parser::Vec3f &point);
    parser::Vec3f get_ks_modified_phong(parser::Vec3f &ks, parser::Vec3f &normal, parser::Vec3f &light_direction,
                                        parser::Vec3f &cam_position, parser::Vec3f &point);
    parser::Vec3f get_ks_normalized_modified_phong(parser::Vec3f &ks, parser::Vec3f &normal, parser::Vec3f &light_direction,
                                                   parser::Vec3f &cam_position, parser::Vec3f &point);

    //BLINN-PHONG
    parser::Vec3f get_ks_blinnphong(parser::Vec3f &ks, parser::Vec3f &normal, parser::Vec3f &light_direction,
                                    parser::Vec3f &cam_position, parser::Vec3f &point);
    parser::Vec3f get_ks_modified_blinnphong(parser::Vec3f &ks, parser::Vec3f &normal,parser::Vec3f &light_direction,
                                             parser::Vec3f &cam_position, parser::Vec3f &point);
    parser::Vec3f get_ks_normalized_modified_blinnphong(parser::Vec3f &ks, parser::Vec3f &normal,parser::Vec3f &light_direction,
                                                        parser::Vec3f &cam_position, parser::Vec3f &point);

    parser::Vec3f get_ks_torrance_sparrow(parser::Vec3f &ks, parser::Vec3f &normal,parser::Vec3f &light_direction,
                                          parser::Vec3f &cam_position, parser::Vec3f &point, float refraction_index);

    float distribution_function(float alpha);
    float get_geometry_term(parser::Vec3f &wi, parser::Vec3f &wo, parser::Vec3f &normal, parser::Vec3f &half_vector);
    float get_fresnel_reflectance(float refraction_index, parser::Vec3f &wi, parser::Vec3f &half_vector);

    parser::Vec3f get_kd(const parser::Vec3f &kd, const parser::Vec3f &normal, const parser::Vec3f &light_direction);
    parser::Vec3f get_kd_pi(parser::Vec3f &kd, const parser::Vec3f &normal, const parser::Vec3f &light_direction);
};


#endif
