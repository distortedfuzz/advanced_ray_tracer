#include "texture.h"


texture::texture(int image_id,int normalizer, std::vector<parser::Image> &images, int decal_mode, int interpolation,
        int noise_conversion, float bump_factor, float noise_scale, int num_octaves, int tex_type,float scale,
                 float offset, parser::Vec3f &black_color, parser::Vec3f &white_color){

    this->image_id = image_id;
    this->decal_mode = decal_mode;
    this->interpolation = interpolation;
    this->noise_conversion = noise_conversion;
    this->bump_factor = bump_factor;
    this->noise_scale = noise_scale;
    this->num_octaves = num_octaves;
    this->normalizer = normalizer;
    this->texture_type = tex_type;
    this->scale = scale;
    this->offset = offset;
    this->black_color = black_color;
    this->white_color = white_color;



    if(tex_type == 0){
        this->width = images[image_id-1].width;
        this->height = images[image_id-1].height;
        this->channels = images[image_id-1].channels;
        this->image_data = images[image_id-1].data;
        this->image_data_exr = images[image_id-1].exr_data;

        if(images[image_id-1].is_exr){
            this->is_exr = true;
        }else{
            this->is_exr = false;
        }
    }


    if(tex_type == 1){

        this->gradients.push_back(parser::Vec3f{1,  1,  0});
        this->gradients.push_back(parser::Vec3f{-1,  1,  0});
        this->gradients.push_back(parser::Vec3f{1, -1,  0});
        this->gradients.push_back(parser::Vec3f{-1, -1,  0});

        this->gradients.push_back(parser::Vec3f{1,  0,  1});
        this->gradients.push_back(parser::Vec3f{-1,  0,  1});
        this->gradients.push_back(parser::Vec3f{1,  0, -1});
        this->gradients.push_back(parser::Vec3f{-1,  0, -1});

        this->gradients.push_back(parser::Vec3f{0,  1,  1});
        this->gradients.push_back(parser::Vec3f{0, -1,  1});
        this->gradients.push_back(parser::Vec3f{ 0,  1, -1});
        this->gradients.push_back(parser::Vec3f{0, -1, -1});

        this->gradients.push_back(parser::Vec3f{1.41, 0,0});
        this->gradients.push_back(parser::Vec3f{0, 1.41,0});
        this->gradients.push_back(parser::Vec3f{0, 0,1.41});

        this->gradients.push_back(parser::Vec3f{-1.41, 0,0});
        this->gradients.push_back(parser::Vec3f{0, -1.41,0});
        this->gradients.push_back(parser::Vec3f{0, 0,-1.41});
    }

}




parser::Vec3f texture::get_texture_value_nearest(parser::Vec2f &coordinate){

    int pixel_x = (std::round(coordinate.u * (width - 1)));
    int pixel_y = (std::round(coordinate.v * (height - 1)));

    pixel_x = std::clamp(pixel_x, 0, width - 1);
    pixel_y = std::clamp(pixel_y, 0, height - 1);


    parser::Vec3f result;


    if(!is_exr){
        int index = (pixel_y * width + pixel_x) * channels;
        result.x = image_data[index];
        result.y = image_data[index + 1];
        result.z = image_data[index + 2];
    }else{
        int index = (pixel_y * width + pixel_x) * 4;
        result.x = image_data_exr[index];
        result.y = image_data_exr[index + 1];
        result.z = image_data_exr[index + 2];
    }


    return result;
}

parser::Vec3f texture::get_texture_value_bilinear(parser::Vec2f &coordinate){


    float x = coordinate.u * (width - 1);
    float y = coordinate.v * (height - 1);

    int p = std::floor(x);
    int q = std::floor(y);

    float dx = x - p;
    float dy = y - q;

    p = std::clamp(p, 0, width - 2);
    q = std::clamp(q, 0, height - 2);

    parser::Vec3f result;

    if(!is_exr){
        result.x = image_data[(q * width + p) * channels + 0] * (1 - dx) * (1 - dy) +
                   image_data[(q * width + (p + 1)) * channels + 0] * dx * (1 - dy) +
                   image_data[((q + 1) * width + p) * channels + 0] * (1 - dx) * dy +
                   image_data[((q + 1) * width + (p + 1)) * channels + 0] * dx * dy;

        result.y = image_data[(q * width + p) * channels + 1] * (1 - dx) * (1 - dy) +
                   image_data[(q * width + (p + 1)) * channels + 1] * dx * (1 - dy) +
                   image_data[((q + 1) * width + p) * channels + 1] * (1 - dx) * dy +
                   image_data[((q + 1) * width + (p + 1)) * channels + 1] * dx * dy;


        result.z = image_data[(q * width + p) * channels + 2] * (1 - dx) * (1 - dy) +
                   image_data[(q * width + (p + 1)) * channels + 2] * dx * (1 - dy) +
                   image_data[((q + 1) * width + p) * channels + 2] * (1 - dx) * dy +
                   image_data[((q + 1) * width + (p + 1)) * channels + 2] * dx * dy;

    }else{
        result.x = (image_data_exr[(q * width + p) * 4 + 0])  * (1 - dx) * (1 - dy) +
                    (image_data_exr[(q * width + (p + 1)) * 4 + 0])  * dx * (1 - dy) +
                    (image_data_exr[((q + 1) * width + p) * 4 + 0])  * (1 - dx) * dy +
                    (image_data_exr[((q + 1) * width + (p + 1)) * 4 + 0])  * dx * dy;

        result.y = (image_data_exr[(q * width + p) * 4 + 1])  * (1 - dx) * (1 - dy) +
                    (image_data_exr[(q * width + (p + 1)) * 4 + 1])  * dx * (1 - dy) +
                    (image_data_exr[((q + 1) * width + p) * 4 + 1])  * (1 - dx) * dy +
                    (image_data_exr[((q + 1) * width + (p + 1)) * 4 + 1])  * dx * dy;


        result.z = (image_data_exr[(q * width + p) * 4 + 2])  * (1 - dx) * (1 - dy) +
                    (image_data_exr[(q * width + (p + 1)) * 4 + 2])  * dx * (1 - dy) +
                    (image_data_exr[((q + 1) * width + p) * 4 + 2])  * (1 - dx) * dy +
                    (image_data_exr[((q + 1) * width + (p + 1)) * 4 + 2])  * dx * dy;

    }



    return result;

}

parser::Vec3f texture::get_texture_value(parser::Vec2f &coordinate){

    //image
    if(texture_type == 0){

        if(interpolation == 0){
            return get_texture_value_nearest(coordinate);
        }else if(interpolation == 1){
            return get_texture_value_bilinear(coordinate);
        }else{
            return parser::Vec3f{0,0,0};
        }

    }


}

parser::Vec3f texture::get_texture_value_procedural(parser::Vec3f &coordinate){

    if(texture_type == 1){

        return get_texture_value_perlin(coordinate);

    }else if(texture_type == 2){
        return get_texture_value_checkerboard(coordinate);
    }


}

parser::Vec3f texture::get_texture_value_perlin(parser::Vec3f &coordinate){

    float weight = 1;
    float frequency = 1;
    parser::Vec3f result{0,0,0};

    for(int i = 0; i < num_octaves; i++){
        float x = coordinate.x * noise_scale * frequency;
        float y = coordinate.y * noise_scale * frequency;
        float z = coordinate.z * noise_scale * frequency;

        parser::Vec3i c1{(int)floor(x), (int)floor(y), (int)floor(z)};
        parser::Vec3i c2{(int)floor(x+1), (int)floor(y), (int)floor(z)};
        parser::Vec3i c3{(int)floor(x), (int)floor(y+1), (int)floor(z)};
        parser::Vec3i c4{(int)floor(x+1), (int)floor(y+1), (int)floor(z)};

        parser::Vec3i c5{(int)floor(x), (int)floor(y), (int)floor(z+1)};
        parser::Vec3i c6{(int)floor(x+1), (int)floor(y), (int)floor(z+1)};
        parser::Vec3i c7{(int)floor(x), (int)floor(y+1), (int)floor(z+1)};
        parser::Vec3i c8{(int)floor(x+1), (int)floor(y+1), (int)floor(z+1)};

        parser::Vec3f grad_vec1 = normalize_vector(gradients[hash_perlin(c1.x, c1.y, c1.z, 18)]);
        parser::Vec3f grad_vec2 = normalize_vector(gradients[hash_perlin(c2.x, c2.y, c2.z,18)]);
        parser::Vec3f grad_vec3 = normalize_vector(gradients[hash_perlin(c3.x, c3.y, c3.z,18)]);
        parser::Vec3f grad_vec4 = normalize_vector(gradients[hash_perlin(c4.x, c4.y, c4.z,18)]);

        parser::Vec3f grad_vec5 = normalize_vector(gradients[hash_perlin(c5.x, c5.y, c5.z,18)]);
        parser::Vec3f grad_vec6 = normalize_vector(gradients[hash_perlin(c6.x, c6.y, c6.z,18)]);
        parser::Vec3f grad_vec7 = normalize_vector(gradients[hash_perlin(c7.x, c7.y, c7.z,18)]);
        parser::Vec3f grad_vec8 = normalize_vector(gradients[hash_perlin(c8.x, c8.y, c8.z,18)]);

        parser::Vec3f corner_vec1{x - c1.x, y - c1.y, z - c1.z};
        parser::Vec3f corner_vec2{x - c2.x, y - c2.y, z - c2.z};
        parser::Vec3f corner_vec3{x - c3.x, y - c3.y, z - c3.z};
        parser::Vec3f corner_vec4{x - c4.x, y - c4.y, z - c4.z};

        parser::Vec3f corner_vec5{x - c5.x, y - c5.y, z - c5.z};
        parser::Vec3f corner_vec6{x - c6.x, y - c6.y, z - c6.z};
        parser::Vec3f corner_vec7{x - c7.x, y - c7.y, z - c7.z};
        parser::Vec3f corner_vec8{x - c8.x, y - c8.y, z - c8.z};

        float dot1 = dot_product(grad_vec1, corner_vec1);
        float dot2 = dot_product(grad_vec2, corner_vec2);
        float dot3 = dot_product(grad_vec3, corner_vec3);
        float dot4 = dot_product(grad_vec4, corner_vec4);

        float dot5 = dot_product(grad_vec5, corner_vec5);
        float dot6 = dot_product(grad_vec6, corner_vec6);
        float dot7 = dot_product(grad_vec7, corner_vec7);
        float dot8 = dot_product(grad_vec8, corner_vec8);

        parser::Vec3f diff_c1 = {(x - (int)floor(x)), fabs(y - (int)floor(y)), fabs(z - (int)floor(z))};
        parser::Vec3f diff_c2 = {(x - (int)floor(x+1)), fabs(y - (int)floor(y)), fabs(z - (int)floor(z))};
        parser::Vec3f diff_c3 = {(x - (int)floor(x)), fabs(y - (int)floor(y+1)), fabs(z - (int)floor(z))};
        parser::Vec3f diff_c4 = {(x - (int)floor(x+1)), fabs(y - (int)floor(y+1)), fabs(z - (int)floor(z))};

        parser::Vec3f diff_c5 = {(x - (int)floor(x)), fabs(y - (int)floor(y)), fabs(z - (int)floor(z+1))};
        parser::Vec3f diff_c6 = {(x - (int)floor(x+1)), fabs(y - (int)floor(y)), fabs(z - (int)floor(z+1))};
        parser::Vec3f diff_c7 = {(x - (int)floor(x)), fabs(y - (int)floor(y+1)), fabs(z - (int)floor(z+1))};
        parser::Vec3f diff_c8 = {(x - (int)floor(x+1)), fabs(y - (int)floor(y+1)), fabs(z - (int)floor(z+1))};

        float weight_1 = weight_perlin(diff_c1.x) * weight_perlin(diff_c1.y) * weight_perlin(diff_c1.z);
        float weight_2 = weight_perlin(diff_c2.x) * weight_perlin(diff_c2.y) * weight_perlin(diff_c2.z);
        float weight_3 = weight_perlin(diff_c3.x) * weight_perlin(diff_c3.y) * weight_perlin(diff_c3.z);
        float weight_4 = weight_perlin(diff_c4.x) * weight_perlin(diff_c4.y) * weight_perlin(diff_c4.z);

        float weight_5 = weight_perlin(diff_c5.x) * weight_perlin(diff_c5.y) * weight_perlin(diff_c5.z);
        float weight_6 = weight_perlin(diff_c6.x) * weight_perlin(diff_c6.y) * weight_perlin(diff_c6.z);
        float weight_7 = weight_perlin(diff_c7.x) * weight_perlin(diff_c7.y) * weight_perlin(diff_c7.z);
        float weight_8 = weight_perlin(diff_c8.x) * weight_perlin(diff_c8.y) * weight_perlin(diff_c8.z);

        float weighted = weight_1 * dot1 + weight_2 * dot2 + weight_3 * dot3 + weight_4 * dot4 +
                         weight_5 * dot5 + weight_6 * dot6 + weight_7 * dot7 + weight_8 * dot8;

        float noise_value = 0.0;
        if(noise_conversion == 0){
            noise_value = fabs(weighted);
        }else if(noise_conversion == 1){
            noise_value = (weighted + 1.0) / 2.0;
        }

        result = vector_add(result, vector_multiply(parser::Vec3f{noise_value, noise_value, noise_value}, weight));

        frequency *= 2;
        weight /= 2;
    }


    return result;

}

parser::Vec3f texture::get_texture_value_checkerboard(parser::Vec3f &coordinate){

    bool x = (int) ((coordinate.x + offset + 1e-5) * scale) % 2;
    x = x || (int) ((coordinate.x + offset - 1e-5) * scale) % 2;
    bool y = (int) ((coordinate.y + offset + 1e-5) * scale) % 2;
    y = y || (int) ((coordinate.y + offset - 1e-5) * scale) % 2;
    bool z = (int) ((coordinate.z + offset + 1e-5) * scale) % 2;
    z = z || (int) ((coordinate.z + offset - 1e-5) * scale) % 2;

    bool xorXY;
    if(x == y){
        xorXY = false;
    }else{
        xorXY = true;
    }

    if(xorXY != z){
        return black_color;
    }else{
        return white_color;
    }

}


parser::Vec3f texture::get_specific_pixel(parser::Vec2i coordinate){


    int index = (coordinate.v * width + coordinate.u) * channels;

    parser::Vec3f result;


    result.x = image_data[index];
    result.y = image_data[index + 1];
    result.z = image_data[index + 2];

    return result;

}
