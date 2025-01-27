#include "BRDF.h"


BRDF::BRDF(int mode, int id, bool normalized, float exponent, bool kd_fresnel){
    this->mode = mode;
    this->id = id;
    this->normalized = normalized;
    this->exponent = exponent;
    this->kd_fresnel = kd_fresnel;
}


int BRDF::get_id(){
    return this->id;
}


int BRDF::get_mode(){
    return this->mode;
}


bool BRDF::is_normalized(){
    return this->normalized;
}


float BRDF::get_exponent(){
    return this->exponent;
}

parser::Vec3f BRDF::get_all_result(parser::Vec3f &kd,parser::Vec3f &ks, parser::Vec3f &normal, parser::Vec3f &light_direction,
                         parser::Vec3f &cam_position, parser::Vec3f &point, float refraction_index){

    parser::Vec3f result{0,0,0};
    /*
    * 0 : Original Blinn-Phong
    * 1 : Modified Blinn-Phong
    * 2 : Original Phong
    * 3 : Modified Phong
    * 4 : Torrance Sparrow
    */
    if(this->mode == 0){
        float cos_theta_i = dot_product(normalize_vector(normal), normalize_vector(light_direction));
        if(acos(cos_theta_i) > M_PI/2){
            return parser::Vec3f{0,0,0};
        }

        result = vector_add(get_kd(kd, normal, light_direction), result);
        result = vector_add(get_ks_blinnphong(ks, normal, light_direction,
                                              cam_position, point), result);

    }else if(this->mode == 1){
        float cos_theta_i = dot_product(normalize_vector(normal), normalize_vector(light_direction));
        if(acos(cos_theta_i) > M_PI/2){
            return parser::Vec3f{0,0,0};
        }

        if(!this->normalized){

            result = vector_add(get_kd(kd, normal, light_direction), result);
            result = vector_add(get_ks_modified_blinnphong(ks, normal, light_direction,
                                                                 cam_position, point), result);

        }else{

            result = vector_add(get_kd_pi(kd, normal, light_direction), result);
            result = vector_add(get_ks_normalized_modified_blinnphong(ks, normal, light_direction,
                                                           cam_position, point), result);

        }

    }else if(this->mode == 2){

        float cos_theta_i = dot_product(normalize_vector(normal), normalize_vector(light_direction));
        if(acos(cos_theta_i) > M_PI/2){
            return parser::Vec3f{0,0,0};
        }
        result = vector_add(get_kd(kd, normal, light_direction), result);
        result = vector_add(get_ks_phong(ks, normal, light_direction,
                                         cam_position, point), result);

    }else if(this->mode == 3){
        float cos_theta_i = dot_product(normalize_vector(normal), normalize_vector(light_direction));
        if(acos(cos_theta_i) > M_PI/2){
            return parser::Vec3f{0,0,0};
        }

        if(!this->normalized){

            result = vector_add(get_kd(kd, normal, light_direction), result);
            result = vector_add(get_ks_modified_phong(ks, normal, light_direction,
                                                      cam_position, point), result);

        }else{

            result = vector_add(get_kd_pi(kd, normal, light_direction), result);
            result = vector_add(get_ks_normalized_modified_phong(ks, normal, light_direction,
                                                                 cam_position, point), result);

        }


    }else if(this->mode == 4){
        float cos_theta_i = dot_product(normalize_vector(normal), normalize_vector(light_direction));
        if(acos(cos_theta_i) > M_PI/2){
            return parser::Vec3f{0,0,0};
        }

        if(!kd_fresnel){
            result = vector_add(get_kd_pi(kd, normal, light_direction), result);
        }else{
            parser::Vec3f cam_vector = normalize_vector(vector_subtract(cam_position, point));
            parser::Vec3f half_vector = normalize_vector(vector_add(light_direction, cam_vector));
            float fresnel = get_fresnel_reflectance(refraction_index, cam_vector, half_vector);

            result = vector_add(vector_multiply(get_kd_pi(kd, normal, light_direction), 1-fresnel), result);
        }

        result = vector_add(get_ks_torrance_sparrow(ks, normal, light_direction,
                                                    cam_position, point, refraction_index), result);

    }


    return result;
}

parser::Vec3f BRDF::get_kd(const parser::Vec3f &kd, const parser::Vec3f &normal, const parser::Vec3f &light_direction) {


    float cost = dot_product(normal, light_direction)/ (get_vector_magnitude(normal)* get_vector_magnitude(light_direction));
    cost = fmax(cost, 0.0f);

    return vector_multiply(kd, cost);
}

parser::Vec3f BRDF::get_kd_pi(parser::Vec3f &kd, const parser::Vec3f &normal, const parser::Vec3f &light_direction){
    float cost = dot_product(normal, light_direction)/ (get_vector_magnitude(normal)* get_vector_magnitude(light_direction));
    cost = fmax(cost, 0.008f);

    return vector_multiply(kd, cost / M_PI);
}



parser::Vec3f BRDF::get_ks_phong(parser::Vec3f &ks, parser::Vec3f &normal, parser::Vec3f &light_direction,
                                 parser::Vec3f &cam_position, parser::Vec3f &point){


    parser::Vec3f wo = normalize_vector(vector_subtract(cam_position, point));
    parser::Vec3f reflection_ray = normalize_vector(vector_subtract(
            vector_multiply(normal, 2.0f * dot_product(normal, light_direction)), light_direction));

    float cos_alpha_r = std::fmax(0.0f, dot_product(reflection_ray, wo));

    parser::Vec3f result;
    result.x = ks.x * pow(cos_alpha_r, exponent);
    result.y = ks.y * pow(cos_alpha_r, exponent);
    result.z = ks.z * pow(cos_alpha_r, exponent);

    return result;
}


parser::Vec3f BRDF::get_ks_modified_phong(parser::Vec3f &ks, parser::Vec3f &normal, parser::Vec3f &light_direction,
                                          parser::Vec3f &cam_position, parser::Vec3f &point){


    parser::Vec3f wo = normalize_vector(vector_subtract(cam_position, point));
    parser::Vec3f reflection_ray = normalize_vector(vector_subtract(
            vector_multiply(normal, 2.0f * dot_product(normal, light_direction)), light_direction));

    float cos_alpha_r = std::fmax(0.0f, dot_product(reflection_ray, wo));

    parser::Vec3f result;

    float cos_theta_i = dot_product(normal, light_direction);
    result.x = ks.x * pow(cos_alpha_r, exponent) * cos_theta_i;
    result.y = ks.y * pow(cos_alpha_r, exponent) * cos_theta_i;
    result.z = ks.z * pow(cos_alpha_r, exponent) * cos_theta_i;

    return result;

}


parser::Vec3f BRDF::get_ks_normalized_modified_phong(parser::Vec3f &ks, parser::Vec3f &normal, parser::Vec3f &light_direction,
                                                     parser::Vec3f &cam_position, parser::Vec3f &point){

    parser::Vec3f wo = normalize_vector(vector_subtract(cam_position, point));
    parser::Vec3f reflection_ray = normalize_vector(vector_subtract(
            vector_multiply(normal, 2.0f * dot_product(normal, light_direction)), light_direction));

    float cos_alpha_r = std::fmax(0.0f, dot_product(reflection_ray, wo));

    float pi_op = (exponent + 2.0f) / (2.0f * M_PI);
    parser::Vec3f result;
    float cos_theta_i = dot_product(normal, light_direction);
    result.x = ks.x * pow(cos_alpha_r, exponent) * pi_op * cos_theta_i;
    result.y = ks.y * pow(cos_alpha_r, exponent) * pi_op * cos_theta_i;
    result.z = ks.z * pow(cos_alpha_r, exponent) * pi_op * cos_theta_i;

    return result;

}


parser::Vec3f BRDF::get_ks_blinnphong(parser::Vec3f &ks, parser::Vec3f &normal, parser::Vec3f &light_direction,
                                      parser::Vec3f &cam_position, parser::Vec3f &point) {

    parser::Vec3f cam_vector = normalize_vector(vector_subtract(cam_position, point));

    parser::Vec3f half_vector = normalize_vector(vector_add(light_direction, cam_vector));

    float cosa = std::fmax(0.0f, dot_product(normal, half_vector));

    parser::Vec3f result{0,0,0};

    result.x = ks.x * pow(cosa, exponent);
    result.y = ks.y * pow(cosa, exponent);
    result.z = ks.z * pow(cosa, exponent);


    return result;

}


parser::Vec3f BRDF::get_ks_modified_blinnphong(parser::Vec3f &ks, parser::Vec3f &normal,parser::Vec3f &light_direction,
                                               parser::Vec3f &cam_position, parser::Vec3f &point){

    parser::Vec3f cam_vector = normalize_vector(vector_subtract(cam_position, point));

    parser::Vec3f half_vector = normalize_vector(vector_add(light_direction, cam_vector));

    float cosa = dot_product(normal, half_vector);
    float cos_theta_i = dot_product(normal, light_direction);
    parser::Vec3f result {0,0,0};

    result.x = ks.x * pow(cosa, exponent) * cos_theta_i;
    result.y = ks.y * pow(cosa, exponent) * cos_theta_i;
    result.z = ks.z * pow(cosa, exponent) * cos_theta_i;

    return result;

}


parser::Vec3f BRDF::get_ks_normalized_modified_blinnphong(parser::Vec3f &ks, parser::Vec3f &normal,parser::Vec3f &light_direction,
                                                          parser::Vec3f &cam_position, parser::Vec3f &point){

    parser::Vec3f cam_vector = normalize_vector(vector_subtract(cam_position, point));

    parser::Vec3f half_vector = normalize_vector(vector_add(light_direction, cam_vector));
    float cosa = dot_product(normal, half_vector)/ (get_vector_magnitude(normal)* get_vector_magnitude(half_vector));

    parser::Vec3f result {0,0,0};

    float cos_theta_i = dot_product(normal, light_direction);
    float pi_op = (exponent + 8.0) / (8.0 * M_PI);
    result.x = ks.x * pow(cosa, exponent) * pi_op * cos_theta_i;
    result.y = ks.y * pow(cosa, exponent) * pi_op * cos_theta_i;
    result.z = ks.z * pow(cosa, exponent) * pi_op * cos_theta_i;

    return result;

}

parser::Vec3f BRDF::get_ks_torrance_sparrow(parser::Vec3f &ks, parser::Vec3f &normal,parser::Vec3f &light_direction,
                                            parser::Vec3f &cam_position, parser::Vec3f &point, float refraction_index){

    parser::Vec3f cam_vector = normalize_vector(vector_subtract(cam_position, point));

    parser::Vec3f half_vector = normalize_vector(vector_add(light_direction, cam_vector));

    light_direction = normalize_vector(light_direction);
    normal = normalize_vector(normal);

    float alpha = dot_product(half_vector, normal);

    float distribution = distribution_function(alpha);

    float geometry_term = get_geometry_term(light_direction ,cam_vector, normal, half_vector);

    float fresnel_reflectance = get_fresnel_reflectance(refraction_index, cam_vector, half_vector);

    float upper = distribution * geometry_term * fresnel_reflectance;

    float lower = 4 * dot_product(normal, light_direction) * dot_product(normal, cam_vector);

    float cos_theta_i = dot_product(normal, light_direction);
    return vector_multiply(ks, cos_theta_i * upper / (lower));
}

float BRDF::distribution_function(float alpha){
    return (pow((alpha), exponent) * (exponent + 2)) / (2*M_PI);
}

float BRDF::get_geometry_term(parser::Vec3f &wi, parser::Vec3f &wo, parser::Vec3f &normal, parser::Vec3f &half_vector){

    float wo_inner_term;
    float wi_inner_term;

    wo_inner_term = 2 * (dot_product(normal, half_vector)) * (dot_product(normal, wo))/
                    dot_product(wo,half_vector);

    wi_inner_term = 2 * (dot_product(normal, half_vector)) * (dot_product(normal, wi))/
                    dot_product(wo,half_vector);

    float inner_min = std::min(wo_inner_term, wi_inner_term);

    float outer_min = std::min(1.0f, inner_min);

    return outer_min;
}

float BRDF::get_fresnel_reflectance(float refraction_index, parser::Vec3f &cam_vector, parser::Vec3f &half_vector){

    float r0 = pow((refraction_index - 1.0f) / (refraction_index + 1.0f), 2.0f);

    float dot = dot_product(cam_vector, half_vector);

    return r0 + ((1.0f-r0) * (pow(1.0f - dot, 5.0f)));
}