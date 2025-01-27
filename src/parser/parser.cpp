#include "parser.h"
#include "tinyxml2.h"
#include <sstream>
#include <stdexcept>
#include <cassert>
#include "../ply/happly.h"
#define STB_IMAGE_IMPLEMENTATION
#include "../write/stb_image.h"


void parser::Scene::loadFromXml(const std::string &filepath)
{
    tinyxml2::XMLDocument file;
    std::istringstream istream;


    auto res = file.LoadFile(filepath.c_str());
    if (res)
    {
        throw std::runtime_error("Error: The xml file cannot be loaded.");
    }

    auto root = file.FirstChild();
    if (!root)
    {
        throw std::runtime_error("Error: Root is not found.");
    }


    //Get BackgroundColor
    auto element = root->FirstChildElement("BackgroundColor");
    if (element)
    {
        istream.str(element->GetText());
    }
    else
    {
        istream.str("0 0 0");
    }
    istream >> background_color.x >> background_color.y >> background_color.z;



    istream.clear();
    istream.seekg(0, std::ios::beg);


    //Get ShadowRayEpsilon
    element = root->FirstChildElement("ShadowRayEpsilon");
    if (element)
    {
        istream.str(element->GetText());
    }
    else
    {
        istream .str("0.0001");
    }
    istream >> shadow_ray_epsilon;



    istream.clear();
    istream.seekg(0, std::ios::beg);


    //Get IntersectionTestEpsilon
    element = root->FirstChildElement("IntersectionTestEpsilon");
    if (element)
    {
        istream.str(element->GetText());
    }
    else
    {
        istream.str("0.000001");
    }
    istream >> intersection_test_epsilon;



    istream.clear();
    istream.seekg(0, std::ios::beg);


    //Get MaxRecursionDepth
    element = root->FirstChildElement("MaxRecursionDepth");
    if (element)
    {
        istream.str(element->GetText());
    }
    else
    {
        istream.str("0");
    }
    istream >> max_recursion_depth;

    //Get MinRecursionDepth
    element = root->FirstChildElement("MinRecursionDepth");
    if (element)
    {
        istream.str(element->GetText());
    }
    else
    {
        istream.str("0");
    }
    istream >> min_recursion_depth;



    istream.clear();
    istream.seekg(0, std::ios::beg);



    //Get Cloud Parameters
    element = root->FirstChildElement("CloudParameters");
    if (element)
    {
        istream.str(element->GetText());
        float param = 0;
        while (istream >> param) {
            cloud_parameters.push_back(param);
        }
    }
    else
    {
        cloud_parameters.push_back(0);
        cloud_parameters.push_back(0);
        cloud_parameters.push_back(0);
    }

    istream.clear();
    istream.seekg(0, std::ios::beg);




    //Get Transformations**************************************************************************************************
    element = root->FirstChildElement("Transformations");

    if(element){
        auto child = element->FirstChildElement("Translation");
        Translation translation1;
        while (child)
        {
            translation1.id = atoi(child->Attribute("id"));
            istream.str(child->GetText());
            istream >> translation1.translation_x >> translation1.translation_y>> translation1.translation_z;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            translations.push_back(translation1);
            child = child->NextSiblingElement("Translation");

        }

        child = element->FirstChildElement("Scaling");
        Scaling scaling1;
        while (child)
        {
            scaling1.id = atoi(child->Attribute("id"));
            istream.str(child->GetText());
            istream >> scaling1.scaling_x >> scaling1.scaling_y>> scaling1.scaling_z;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            scalings.push_back(scaling1);
            child = child->NextSiblingElement("Scaling");

        }

        child = element->FirstChildElement("Rotation");
        Rotation rotation1;
        while (child)
        {

            rotation1.id = atoi(child->Attribute("id"));
            istream.str(child->GetText());
            istream >> rotation1.angle >> rotation1.rotation_x>> rotation1.rotation_y >> rotation1.rotation_z;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            rotations.push_back(rotation1);
            child = child->NextSiblingElement("Rotation");

        }

        child = element->FirstChildElement("Composite");
        Composite composite1;
        while (child)
        {
            Vec4f row1;
            Vec4f row2;
            Vec4f row3;
            Vec4f row4;
            composite1.id = atoi(child->Attribute("id"));
            istream.str(child->GetText());
            istream >> row1.x >> row1.y>> row1.z >> row1.w;
            istream >> row2.x >> row2.y>> row2.z >> row2.w;
            istream >> row3.x >> row3.y>> row3.z >> row3.w;
            istream >> row4.x >> row4.y>> row4.z >> row4.w;

            std::vector<Vec4f> turn;
            turn.push_back(row1);
            turn.push_back(row2);
            turn.push_back(row3);
            turn.push_back(row4);

            for(int i = 0; i < 4 ; i++){
                std::cout<<"adding values to matrix:"<<std::endl;
                std::cout<<turn[i].x<<" "<<turn[i].y<<" "<<turn[i].z<<" "<<turn[i].w<<std::endl;
                composite1.elements.push_back(turn[i].x);
                composite1.elements.push_back(turn[i].y);
                composite1.elements.push_back(turn[i].z);
                composite1.elements.push_back(turn[i].w);

            }
            istream.clear();
            istream.seekg(0, std::ios::beg);

            composites.push_back(composite1);
            composite1.elements.clear();
            child = child->NextSiblingElement("Composite");

        }

    }

    //*********************************************************************************************************************



    //Get Lights
    //ambient
    element = root->FirstChildElement("Lights");
    tinyxml2::XMLElement* child;
    if(element != NULL) {
        child = element->FirstChildElement("AmbientLight");
        if (child) {
            istream.str(child->GetText());
            istream >> ambient_light.x >> ambient_light.y >> ambient_light.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        } else {
            ambient_light.x = 0.0;
            ambient_light.y = 0.0;
            ambient_light.z = 0.0;
        }



        //point
        element = element->FirstChildElement("PointLight");
        PointLight point_light;
        while (element) {
            child = element->FirstChildElement("Position");
            istream.str(child->GetText());
            istream >> point_light.position.x >> point_light.position.y >> point_light.position.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);


            child = element->FirstChildElement("Intensity");
            istream.str(child->GetText());
            istream >> point_light.intensity.x >> point_light.intensity.y >> point_light.intensity.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);


            //Transformations************************************************************************************
            child = element->FirstChildElement("Transformations");
            if (child) {
                istream.str(child->GetText());

                std::string singular_transformation;

                while (istream >> singular_transformation) {
                    Transformation new_transformation;
                    if (singular_transformation.at(0) == 't') {
                        new_transformation.transformation_type = 0;
                    } else if (singular_transformation.at(0) == 's') {
                        new_transformation.transformation_type = 1;
                    } else if (singular_transformation.at(0) == 'r') {
                        new_transformation.transformation_type = 2;
                    }

                    new_transformation.id = stoi(singular_transformation.substr(1));
                    point_light.transformations.push_back(new_transformation);
                }


                istream.clear();
                istream.seekg(0, std::ios::beg);

            }
            //***************************************************************************************************


            point_lights.push_back(point_light);
            element = element->NextSiblingElement("PointLight");
        }


        //area
        element = root->FirstChildElement("Lights");
        element = element->FirstChildElement("AreaLight");
        AreaLight area_light;
        while (element) {

            child = element->FirstChildElement("Position");
            istream.str(child->GetText());
            istream >> area_light.position.x >> area_light.position.y >> area_light.position.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            child = element->FirstChildElement("Normal");
            istream.str(child->GetText());
            istream >> area_light.normal.x >> area_light.normal.y >> area_light.normal.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            child = element->FirstChildElement("Size");
            istream.str(child->GetText());
            istream >> area_light.size;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            child = element->FirstChildElement("Radiance");
            istream.str(child->GetText());
            istream >> area_light.radiance.x >> area_light.radiance.y >> area_light.radiance.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            area_lights.push_back(area_light);
            element = element->NextSiblingElement("AreaLight");
        }


        //directional
        element = root->FirstChildElement("Lights");
        element = element->FirstChildElement("DirectionalLight");
        DirectionalLight directional_light;
        while (element) {

            child = element->FirstChildElement("Direction");
            istream.str(child->GetText());
            istream >> directional_light.direction.x >> directional_light.direction.y >> directional_light.direction.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            child = element->FirstChildElement("Radiance");
            istream.str(child->GetText());
            istream >> directional_light.radiance.x >> directional_light.radiance.y >> directional_light.radiance.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);


            directional_lights.push_back(directional_light);
            element = element->NextSiblingElement("DirectionalLight");
        }


        //spot
        element = root->FirstChildElement("Lights");
        element = element->FirstChildElement("SpotLight");
        SpotLight spot_light;
        while (element) {
            /*
            Vec3f position;
            Vec3f direction;
            Vec3f intensity;
            float coverage_angle;
            float falloff_angle;
             */

            child = element->FirstChildElement("Position");
            istream.str(child->GetText());
            istream >> spot_light.position.x >> spot_light.position.y >> spot_light.position.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            child = element->FirstChildElement("Direction");
            istream.str(child->GetText());
            istream >> spot_light.direction.x >> spot_light.direction.y >> spot_light.direction.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            child = element->FirstChildElement("Intensity");
            istream.str(child->GetText());
            istream >> spot_light.intensity.x >> spot_light.intensity.y >> spot_light.intensity.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            child = element->FirstChildElement("CoverageAngle");
            istream.str(child->GetText());
            istream >> spot_light.coverage_angle;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            child = element->FirstChildElement("FalloffAngle");
            istream.str(child->GetText());
            istream >> spot_light.falloff_angle;
            istream.clear();
            istream.seekg(0, std::ios::beg);


            spot_lights.push_back(spot_light);
            element = element->NextSiblingElement("SpotLight");
        }


        //environment
        element = root->FirstChildElement("Lights");
        element = element->FirstChildElement("SphericalDirectionalLight");
        SphericalDirectionalLight spherical_directional_light;
        while (element) {
            /*
            int id
             int type
             int image_id
             */

            spherical_directional_light.id = atoi(element->Attribute("id"));

            if (element->Attribute("type")) {
                std::string type_s = element->Attribute("type");

                spherical_directional_light.type = 0;
                if (type_s == "latlong") {
                    spherical_directional_light.type = 0;
                } else if (type_s == "probe") {
                    spherical_directional_light.type = 1;
                }
            } else {
                spherical_directional_light.type = 0;
            }


            child = element->FirstChildElement("ImageId");
            istream.str(child->GetText());
            istream >> spherical_directional_light.image_id;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            environment_lights.push_back(spherical_directional_light);
            element = element->NextSiblingElement("SphericalDirectionalLight");
        }

    }

    //BRDFSSSSSSSS
    element = root->FirstChildElement("BRDFs");
    BRDF new_brdf;
    if (element){

        auto current_child = element->FirstChildElement("OriginalBlinnPhong");

        while (current_child){

            new_brdf.id = atoi(current_child->Attribute("id"));

            if(current_child->Attribute("kdfresnel")){

                std::string is_normalized = current_child->Attribute("kdfresnel");
                if(is_normalized == "true"){
                    new_brdf.kdfresnel = true;
                }else{
                    new_brdf.kdfresnel = false;
                }

            }else{
                new_brdf.kdfresnel = false;
            }

            if(current_child->Attribute("normalized")){

                std::string is_normalized = current_child->Attribute("normalized");
                if(is_normalized == "true"){
                    new_brdf.normalized = true;
                }else{
                    new_brdf.normalized = false;
                }

            }else{
                new_brdf.normalized = false;
            }

            child = current_child->FirstChildElement("Exponent");
            new_brdf.exponent = atof(child->GetText());
            new_brdf.mode = 0;

            brdfs.push_back(new_brdf);

            current_child = current_child->NextSiblingElement("OriginalBlinnPhong");
        }


        current_child = element->FirstChildElement("ModifiedBlinnPhong");

        while (current_child){

            new_brdf.id = atoi(current_child->Attribute("id"));

            if(current_child->Attribute("kdfresnel")){

                std::string is_normalized = current_child->Attribute("kdfresnel");
                if(is_normalized == "true"){
                    new_brdf.kdfresnel = true;
                }else{
                    new_brdf.kdfresnel = false;
                }

            }else{
                new_brdf.kdfresnel = false;
            }

            if(current_child->Attribute("normalized")){

                std::string is_normalized = current_child->Attribute("normalized");
                if(is_normalized == "true"){
                    new_brdf.normalized = true;
                }else{
                    new_brdf.normalized = false;
                }

            }else{
                new_brdf.normalized = false;
            }

            child = current_child->FirstChildElement("Exponent");
            new_brdf.exponent = atof(child->GetText());
            new_brdf.mode = 1;

            brdfs.push_back(new_brdf);

            current_child = current_child->NextSiblingElement("ModifiedBlinnPhong");
        }


        current_child = element->FirstChildElement("OriginalPhong");

        while (current_child){

            new_brdf.id = atoi(current_child->Attribute("id"));

            if(current_child->Attribute("kdfresnel")){

                std::string is_normalized = current_child->Attribute("kdfresnel");
                if(is_normalized == "true"){
                    new_brdf.kdfresnel = true;
                }else{
                    new_brdf.kdfresnel = false;
                }

            }else{
                new_brdf.kdfresnel = false;
            }
            if(current_child->Attribute("normalized")){

                std::string is_normalized = current_child->Attribute("normalized");
                if(is_normalized == "true"){
                    new_brdf.normalized = true;
                }else{
                    new_brdf.normalized = false;
                }

            }else{
                new_brdf.normalized = false;
            }

            child = current_child->FirstChildElement("Exponent");
            new_brdf.exponent = atof(child->GetText());
            new_brdf.mode = 2;

            brdfs.push_back(new_brdf);

            current_child = current_child->NextSiblingElement("OriginalPhong");
        }



        current_child = element->FirstChildElement("ModifiedPhong");

        while (current_child){

            new_brdf.id = atoi(current_child->Attribute("id"));

            if(current_child->Attribute("kdfresnel")){

                std::string is_normalized = current_child->Attribute("kdfresnel");
                if(is_normalized == "true"){
                    new_brdf.kdfresnel = true;
                }else{
                    new_brdf.kdfresnel = false;
                }

            }else{
                new_brdf.kdfresnel = false;
            }
            if(current_child->Attribute("normalized")){

                std::string is_normalized = current_child->Attribute("normalized");
                if(is_normalized == "true"){
                    new_brdf.normalized = true;
                }else{
                    new_brdf.normalized = false;
                }

            }else{
                new_brdf.normalized = false;
            }

            child = current_child->FirstChildElement("Exponent");
            new_brdf.exponent = atof(child->GetText());
            new_brdf.mode = 3;

            brdfs.push_back(new_brdf);

            current_child = current_child->NextSiblingElement("ModifiedPhong");
        }



        current_child = element->FirstChildElement("TorranceSparrow");

        while (current_child){

            new_brdf.id = atoi(current_child->Attribute("id"));

            if(current_child->Attribute("kdfresnel")){

                std::string is_normalized = current_child->Attribute("kdfresnel");
                if(is_normalized == "true"){
                    new_brdf.kdfresnel = true;
                }else{
                    new_brdf.kdfresnel = false;
                }

            }else{
                new_brdf.kdfresnel = false;
            }

            if(current_child->Attribute("normalized")){

                std::string is_normalized = current_child->Attribute("normalized");
                if(is_normalized == "true"){
                    new_brdf.normalized = true;
                }else{
                    new_brdf.normalized = false;
                }

            }else{
                new_brdf.normalized = false;
            }

            child = current_child->FirstChildElement("Exponent");
            new_brdf.exponent = atof(child->GetText());
            new_brdf.mode = 4;

            brdfs.push_back(new_brdf);

            current_child = current_child->NextSiblingElement("TorranceSparrow");
        }
    }

    //Get Materials
    element = root->FirstChildElement("Materials");
    element = element->FirstChildElement("Material");
    Material material;
    while (element)
    {

        material.is_mirror = (element->Attribute("type", "mirror") != NULL);
        material.is_dielectric = (element->Attribute("type", "dielectric") != NULL);
        material.is_conductor = (element->Attribute("type", "conductor") != NULL);

        if(element->Attribute("BRDF")){
            material.BRDF_id = atoi(element->Attribute("BRDF"));
        }else{
            material.BRDF_id = -1;
        }

        bool is_degamma = false;
        if(element->Attribute("degamma")){
            is_degamma = true;
        }else{
            is_degamma = false;
        }


        child = element->FirstChildElement("AmbientReflectance");
        istream.str(child->GetText());
        istream >> material.ambient.x >> material.ambient.y >> material.ambient.z;
        istream.clear();
        istream.seekg(0, std::ios::beg);


        child = element->FirstChildElement("DiffuseReflectance");
        istream.str(child->GetText());
        istream >> material.diffuse.x >> material.diffuse.y >> material.diffuse.z;
        istream.clear();
        istream.seekg(0, std::ios::beg);


        child = element->FirstChildElement("SpecularReflectance");
        istream.str(child->GetText());
        istream >> material.specular.x >> material.specular.y >> material.specular.z;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        if(is_degamma){
            material.ambient.x = pow(material.ambient.x , 2.2);
            material.ambient.y = pow(material.ambient.y , 2.2);
            material.ambient.z = pow(material.ambient.z , 2.2);

            material.diffuse.x = pow(material.diffuse.x , 2.2);
            material.diffuse.y = pow(material.diffuse.y , 2.2);
            material.diffuse.z = pow(material.diffuse.z , 2.2);

            material.specular.x = pow(material.specular.x , 2.2);
            material.specular.y = pow(material.specular.y , 2.2);
            material.specular.z = pow(material.specular.z , 2.2);
        }


        child = element->FirstChildElement("MirrorReflectance");
        if (child)
        {
            istream.str(child->GetText());
            istream >> material.mirror.x >> material.mirror.y >> material.mirror.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            material.mirror.x = 0.0;
            material.mirror.y = 0.0;
            material.mirror.z = 0.0;
        }


        child = element->FirstChildElement("PhongExponent");
        if(child){
            istream.str(child->GetText());
            istream >> material.phong_exponent;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            material.phong_exponent = 1.0;
        }


        child = element->FirstChildElement("AbsorptionCoefficient");
        if(child)
        {
            istream.str(child->GetText());
            istream >> material.absorption_coefficient.x >> material.absorption_coefficient.y>> material.absorption_coefficient.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            material.absorption_coefficient.x = 0;
            material.absorption_coefficient.y = 0;
            material.absorption_coefficient.z = 0;
        }


        child = element->FirstChildElement("RefractionIndex");
        if(child)
        {
            istream.str(child->GetText());
            istream >> material.refraction_index;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            material.refraction_index = 0;
        }


        child = element->FirstChildElement("AbsorptionIndex");
        if(child)
        {
            istream.str(child->GetText());
            istream >> material.absorption_index;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            material.absorption_index = 0;
        }

        child = element->FirstChildElement("Roughness");
        if(child)
        {
            istream.str(child->GetText());
            istream >> material.roughness;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            material.roughness = 0;
        }


        materials.push_back(material);
        element = element->NextSiblingElement("Material");


    }


    //*****************************************************************************************************************************
    //Textures
    element = root->FirstChildElement("Textures");

    if(element){
        element = element->FirstChildElement("Images");
        if(element){
            element = element->FirstChildElement("Image");

            Image image;
            while (element)
            {

                image.id = atoi(element->Attribute("id"));

                std::filesystem::path xmlDir = std::filesystem::path(filepath).parent_path();
                image.image_route = element->GetText();
                std::filesystem::path full_image_path = xmlDir / image.image_route;
                const char* full_path = full_image_path.c_str();

                size_t len = strlen(image.image_route);
                const char* lastFour = image.image_route + (len - 4);

                if(strcmp(lastFour,".exr") == 0){
                    image.is_exr = true;
                    const char* err = NULL;
                    std::cout<<image.image_route<<std::endl;
                    int ret = LoadEXR(&image.exr_data, &image.width, &image.height, full_path, &err);


                    if (ret != TINYEXR_SUCCESS) {
                        if (err) {
                            fprintf(stderr, "ERR : %s\n", err);
                        }
                    }


                }else{
                    image.is_exr = false;
                    int width, height, channels;

                    image.data = stbi_load(full_path, &width, &height, &channels, 0);
                    image.width = width;
                    image.height = height;
                    image.channels = channels;

                    if (image.data == nullptr) {
                        std::cerr << "Failed to load image: " << stbi_failure_reason() << std::endl;
                    }


                }

                images.push_back(image);
                element = element->NextSiblingElement("Image");
            }
        }

    }


    element = root->FirstChildElement("Textures");
    if(element){
        element = element->FirstChildElement("TextureMap");

        Texture texture;
        while (element)
        {

            texture.id = atoi(element->Attribute("id"));
            std::string type_s = element->Attribute("type");

            texture.texture_type = 0;
            if(type_s == "image"){
                texture.texture_type = 0;
            }else if(type_s == "perlin"){
                texture.texture_type = 1;
            }else if(type_s == "checkerboard"){
                texture.texture_type = 2;
            }else if(type_s == "worley"){
                texture.texture_type = 3;
            }


            child = element->FirstChildElement("ImageId");
            if(child)
            {
                istream.str(child->GetText());
                istream >> texture.image_id;
                istream.clear();
                istream.seekg(0, std::ios::beg);
            }else{
                texture.image_id = -1;
            }


            child = element->FirstChildElement("DecalMode");
            if(child)
            {
                std::string decal_mode_str = child->GetText();
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

                if(decal_mode_str == "replace_kd"){
                    texture.decal_mode = 0;
                }else if(decal_mode_str == "blend_kd"){
                    texture.decal_mode = 1;
                }else if(decal_mode_str == "replace_ks"){
                    texture.decal_mode = 2;
                }else if(decal_mode_str == "replace_background"){
                    texture.decal_mode = 3;
                }else if(decal_mode_str == "replace_normal"){
                    texture.decal_mode = 4;
                }else if(decal_mode_str == "bump_normal"){
                    texture.decal_mode = 5;
                }else if(decal_mode_str == "replace_all"){
                    texture.decal_mode = 6;
                }else{
                    texture.decal_mode = -1;
                }

            }else{
                texture.decal_mode = -1;
            }


            child = element->FirstChildElement("Interpolation");
            if(child)
            {
                std::string interpolation_str = child->GetText();
                /*
                 * interpolation:
                 * 0 = nearest
                 * 1 = bilinear
                 * 2 = trilinear (will not be implemented right now)
                 */
                if(interpolation_str == "nearest"){
                    texture.interpolation = 0;
                }else if(interpolation_str == "bilinear"){
                    texture.interpolation = 1;
                }else if(interpolation_str == "trilinear"){
                    texture.interpolation = 2;
                }else{
                    texture.interpolation = 1;
                }

            }else{
                texture.interpolation = 1;
            }


            child = element->FirstChildElement("NoiseConversion");
            if(child)
            {
                std::string type_str = child->GetText();
                /*
                 * type:
                 * 0 = absvalue
                 * 1 = linear
                 */
                if(type_str == "absval"){
                    texture.noise_conversion = 0;
                }else if(type_str == "linear"){
                    texture.noise_conversion = 1;
                }else{
                    texture.noise_conversion = -1;
                }

            }else{
                texture.noise_conversion = -1;
            }

            child = element->FirstChildElement("Normalizer");
            if(child)
            {
                texture.normalizer = atoi(child->GetText());
            }else{
                texture.normalizer = 255;
            }

            child = element->FirstChildElement("NoiseScale");
            if(child)
            {
                texture.noise_scale = atof(child->GetText());
            }else{
                texture.noise_scale = 1;
            }

            child = element->FirstChildElement("Scale");
            if(child)
            {
                texture.scale = atof(child->GetText());
            }else{
                texture.scale = 0;
            }

            child = element->FirstChildElement("Offset");
            if(child)
            {
                texture.offset = atof(child->GetText());
            }else{
                texture.offset = 0;
            }

            child = element->FirstChildElement("BumpFactor");
            if(child)
            {
                texture.bump_factor = atof(child->GetText());
            }else{
                texture.bump_factor = 1.0;
            }

            child = element->FirstChildElement("NumOctaves");
            if(child)
            {
                texture.num_octaves = atoi(child->GetText());
            }else{
                texture.num_octaves = 1.0;
            }


            child = element->FirstChildElement("BlackColor");
            if(child){
                istream.str(child->GetText());
                istream >> texture.black_color.x >> texture.black_color.y >> texture.black_color.z;
                istream.clear();
                istream.seekg(0, std::ios::beg);
            }else{
                texture.black_color.x = 0.0;
                texture.black_color.y = 0.0;
                texture.black_color.z = 0.0;
            }


            child = element->FirstChildElement("WhiteColor");
            if(child){
                istream.str(child->GetText());
                istream >> texture.white_color.x >> texture.white_color.y >> texture.white_color.z;
                istream.clear();
                istream.seekg(0, std::ios::beg);
            }else{
                texture.white_color.x = 0.0;
                texture.white_color.y = 0.0;
                texture.white_color.z = 0.0;
            }

            child = element->FirstChildElement("GridSize");
            if(child)
            {
                texture.grid_size = atof(child->GetText());
            }else{
                texture.grid_size = 1;
            }


            textures.push_back(texture);
            element = element->NextSiblingElement("TextureMap");
        }
    }



    //*****************************************************************************************************************************

    //Get VertexData
    element = root->FirstChildElement("VertexData");

    if(element){
        istream.str(element->GetText());

        Vec3f vertex;
        while (!(istream>>vertex.x).eof())
        {
            istream >> vertex.y >> vertex.z;

            vertex_data.push_back(vertex);
        }

        istream.clear();
        istream.seekg(0, std::ios::beg);
    }


    std::cout<<"texcoord"<<std::endl;
    //***************************************************************************************************************************
    //Get TexCoordData

    element = root->FirstChildElement("TexCoordData");

    if(element && element->GetText() != nullptr){

        istream.str(element->GetText());

        Vec2f tex_coords;
        while (!(istream>>tex_coords.u).eof())
        {
            istream >> tex_coords.v;

            tex_coord_data.push_back(tex_coords);
        }

        istream.clear();
        istream.seekg(0, std::ios::beg);
    }


    //***************************************************************************************************************************

    std::cout<<"camera"<<std::endl;

    //Get Cameras
    element = root->FirstChildElement("Cameras");
    element = element->FirstChildElement("Camera");
    Camera camera;
    while (element)
    {
        camera.is_look_at = (element->Attribute("type", "lookAt") != NULL);
        camera.handedness = (element->Attribute("handedness", "left") != NULL);

        auto child = element->FirstChildElement("Position");
        istream.str(child->GetText());
        istream >> camera.position.x >> camera.position.y >> camera.position.z;
        istream.clear();
        istream.seekg(0, std::ios::beg);


        if(camera.is_look_at){
            child = element->FirstChildElement("GazePoint");
            if(child){
                istream.str(child->GetText());
                istream >> camera.gaze_point.x >> camera.gaze_point.y >> camera.gaze_point.z;
                istream.clear();
                istream.seekg(0, std::ios::beg);
            }else{
                child = element->FirstChildElement("Gaze");
                istream.str(child->GetText());
                istream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;
                istream.clear();
                istream.seekg(0, std::ios::beg);
                camera.gaze_point.x = INFINITY;
                camera.gaze_point.y = INFINITY;
                camera.gaze_point.z = INFINITY;
            }

        }else{
            child = element->FirstChildElement("Gaze");
            istream.str(child->GetText());
            istream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }



        child = element->FirstChildElement("Up");
        istream.str(child->GetText());
        istream >> camera.up.x >> camera.up.y >> camera.up.z;
        istream.clear();
        istream.seekg(0, std::ios::beg);


        if(camera.is_look_at){
            child = element->FirstChildElement("FovY");
            istream.str(child->GetText());
            istream >> camera.fov_y;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            child = element->FirstChildElement("NearPlane");
            istream.str(child->GetText());
            istream >> camera.near_plane.x >> camera.near_plane.y >> camera.near_plane.z >> camera.near_plane.w;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }



        child = element->FirstChildElement("NearDistance");
        istream.str(child->GetText());
        istream >> camera.near_distance;
        istream.clear();
        istream.seekg(0, std::ios::beg);


        //Transformations************************************************************************************
        child = element->FirstChildElement("Transformations");
        if(child){
            istream.str(child->GetText());

            std::string singular_transformation;

            while(istream >> singular_transformation){
                Transformation new_transformation;
                if(singular_transformation.at(0) == 't'){
                    new_transformation.transformation_type = 0;
                }else if(singular_transformation.at(0) == 's'){
                    new_transformation.transformation_type = 1;
                }else if(singular_transformation.at(0) == 'r'){
                    new_transformation.transformation_type = 2;
                }

                new_transformation.id = stoi(singular_transformation.substr(1));
                camera.transformations.push_back(new_transformation);
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }
        //***************************************************************************************************


        child = element->FirstChildElement("ImageResolution");
        istream.str(child->GetText());
        istream >> camera.image_width >> camera.image_height;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        child = element->FirstChildElement("IgnoreNp");
        if (child){
            istream.str(child->GetText());
            istream >> camera.ignore_np;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            camera.ignore_np = 0;
        }

        child = element->FirstChildElement("NumSamples");
        if (child){
            istream.str(child->GetText());
            istream >> camera.num_samples;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            camera.num_samples = 0;
        }



        child = element->FirstChildElement("ImageName");
        istream.str(child->GetText());
        istream >> camera.image_name;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        child = element->FirstChildElement("FocusDistance");
        if (child){
            istream.str(child->GetText());
            istream >> camera.focus_distance;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            camera.focus_distance = 0.0;
        }

        child = element->FirstChildElement("ApertureSize");
        if (child){
            istream.str(child->GetText());
            istream >> camera.aperture_size;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            camera.aperture_size = 0.0;
        }

        child = element->FirstChildElement("Tonemap");
        if (child){
            auto tonemap_child = child->FirstChildElement("TMO");

            if(strcmp(tonemap_child->GetText(), "Photographic") == 0){
                camera.tone_map.tmo = 0;
            }

            tonemap_child = child->FirstChildElement("TMOOptions");
            istream.str(tonemap_child->GetText());
            istream >> camera.tone_map.key_value >> camera.tone_map.burn_percent;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            tonemap_child = child->FirstChildElement("Saturation");
            istream.str(tonemap_child->GetText());
            istream >> camera.tone_map.saturation;
            istream.clear();
            istream.seekg(0, std::ios::beg);

            tonemap_child = child->FirstChildElement("Gamma");
            istream.str(tonemap_child->GetText());
            istream >> camera.tone_map.gamma;
            istream.clear();
            istream.seekg(0, std::ios::beg);

        }else{
            camera.tone_map.tmo = -1;
        }


        //PATH TRACING ELEMENTS
        child = element->FirstChildElement("Renderer");
        if(child){
            if(strcmp(child->GetText(), "PathTracing") == 0){
                camera.path_tracing = true;
                auto params_child = element->FirstChildElement("RendererParams");

                if(params_child){
                    istream.str(params_child->GetText());
                    std::string param;

                    bool importance_sampling = false;
                    bool nee = false;
                    bool russian_roulette = false;
                    while (istream >> param) {
                        if(param == "ImportanceSampling"){
                            importance_sampling = true;
                        }

                        if(param == "NextEventEstimation"){
                            nee = true;
                        }

                        if(param == "RussianRoulette"){
                            russian_roulette = true;
                        }
                    }

                    if(importance_sampling){
                        camera.path_tracing_params.push_back(true);
                    }else{
                        camera.path_tracing_params.push_back(false);
                    }

                    if(nee){
                        camera.path_tracing_params.push_back(true);
                    }else{
                        camera.path_tracing_params.push_back(false);
                    }

                    if(russian_roulette){
                        camera.path_tracing_params.push_back(true);
                    }else{
                        camera.path_tracing_params.push_back(false);
                    }
                    std::cout<<camera.path_tracing_params[0]<<" "<<
                               camera.path_tracing_params[1]<<" "<<
                               camera.path_tracing_params[2]<<" "<<std::endl;

                    istream.clear();
                    istream.seekg(0, std::ios::beg);
                }else{
                    camera.path_tracing_params.push_back(false);
                    camera.path_tracing_params.push_back(false);
                    camera.path_tracing_params.push_back(false);
                }

                params_child = element->FirstChildElement("SplittingFactor");
                if(params_child){
                    camera.splitting_factor = atoi(params_child->GetText());
                }else{
                    camera.splitting_factor = 1;
                }


            }else{
                camera.path_tracing = false;
                camera.path_tracing_params.push_back(false);
                camera.path_tracing_params.push_back(false);
                camera.path_tracing_params.push_back(false);
                camera.splitting_factor = 0;
            }
        }else{
            camera.path_tracing = false;
            camera.path_tracing_params.push_back(false);
            camera.path_tracing_params.push_back(false);
            camera.path_tracing_params.push_back(false);
            camera.splitting_factor = 0;
        }

        cameras.push_back(camera);
        camera.path_tracing_params.clear();
        element = element->NextSiblingElement("Camera");
    }

    //Get Meshes
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Mesh");
    Mesh mesh;
    int i = 0;
    int mesh_id_difference = 0;
    while (element)
    {
        mesh.is_cloud = false;
        mesh.is_light = false;
        mesh.id = atoi(element->Attribute("id"));
        mesh.texture_ids.clear();
        mesh.transformations.clear();
        child = element->FirstChildElement("Material");
        istream.str(child->GetText());
        istream >> mesh.material_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        mesh.radiance.x = 0.0;
        mesh.radiance.y = 0.0;
        mesh.radiance.z = 0.0;

        //add texture*******************************************************************************************
        child = element->FirstChildElement("Textures");
        if(child != NULL){
            istream.str(child->GetText());

            std::string texture;

            while(istream >> texture){

                mesh.texture_ids.push_back(stoi(texture));
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }



        //Transformations************************************************************************************
        child = element->FirstChildElement("Transformations");
        if(child){
            istream.str(child->GetText());

            std::string singular_transformation;

            while(istream >> singular_transformation){
                Transformation new_transformation;
                if(singular_transformation.at(0) == 't'){
                    new_transformation.transformation_type = 0;
                }else if(singular_transformation.at(0) == 's'){
                    new_transformation.transformation_type = 1;
                }else if(singular_transformation.at(0) == 'r'){
                    new_transformation.transformation_type = 2;
                }else if(singular_transformation.at(0) == 'c'){
                    new_transformation.transformation_type = 3;
                }

                new_transformation.id = stoi(singular_transformation.substr(1));
                mesh.transformations.push_back(new_transformation);
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }


        //***************************************************************************************************
        child = element->FirstChildElement("Faces");
        bool is_ply = false;
        is_ply = (child->Attribute("plyFile") != NULL);
        if(!is_ply){


            if(child->Attribute("vertexOffset")){
                mesh.vertex_offset = atoi(child->Attribute("vertexOffset"));


            }else{
                mesh.vertex_offset = 0;
            }

            if(child->Attribute("textureOffset")){
                std::cout<<"make bigger"<<std::endl;
                mesh.texture_offset = atoi(child->Attribute("textureOffset"));

            }else{
                std::cout<<"make 0"<<std::endl;
                mesh.texture_offset = 0;
            }



            istream.str(child->GetText());

            Face face;
            while (!(istream >> face.v0_id).eof())
            {
                istream >> face.v1_id >> face.v2_id;
                mesh.faces.push_back(face);


            }
        }else{
            std::cout<<"reading ply"<<std::endl;
            int vertex_offset = 0;
            int texture_offset = 0;
            mesh.vertex_offset = vertex_offset;
            mesh.texture_offset = texture_offset;

            int prev_vertex_count = vertex_data.size();
            std::filesystem::path xmlDir = std::filesystem::path(filepath).parent_path();
            const char* ply_file_route = child->Attribute("plyFile");
            std::filesystem::path fullPlyPath = xmlDir / ply_file_route;
            std::string full_path = fullPlyPath;

            char* c_full_path = new char[full_path.size() + 1];
            strcpy(c_full_path, full_path.c_str());

            happly::PLYData plyIn(c_full_path);

            std::vector<std::string> property_names = plyIn.getElement("vertex").getPropertyNames();

            int tex_coord_size = tex_coord_data.size();

            if(property_names.size() == 3 || property_names.size() == 6){
                std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();


                for(auto &vertex:vPos){
                    Vec3f new_vertex{static_cast<float>(vertex[0]), static_cast<float>(vertex[1]),static_cast<float>(vertex[2])};

                    vertex_data.push_back(new_vertex);
                }
            }else if(property_names.size() == 5){

                std::vector<std::array<double, 5>> vPos = plyIn.getVertexPositionsUV();

                for(auto &vertex:vPos){
                    Vec3f new_vertex{static_cast<float>(vertex[0]), static_cast<float>(vertex[1]),static_cast<float>(vertex[2])};
                    Vec2f new_tex_coord{static_cast<float>(vertex[3]), static_cast<float>(vertex[4])};

                    vertex_data.push_back(new_vertex);
                    tex_coord_data.push_back(new_tex_coord);
                }

            }else if(property_names.size() == 8){
                std::vector<std::array<double, 8>> vPos = plyIn.getVertexPositions8();

                for(auto &vertex:vPos){
                    Vec3f new_vertex{static_cast<float>(vertex[0]), static_cast<float>(vertex[1]),static_cast<float>(vertex[2])};
                    Vec2f new_tex_coord{static_cast<float>(vertex[6]), static_cast<float>(vertex[7])};

                    vertex_data.push_back(new_vertex);
                    tex_coord_data.push_back(new_tex_coord);
                }
            }
            std::cout<<"vertex count: "<<vertex_data.size()<<std::endl;


            //*****************************************************************************************
            std::vector<std::vector<int>> fInd;

            // Read face indices
            std::vector<std::vector<int>> faceIndices;
            std::vector<std::vector<unsigned int>> unsigned_faceIndices;

            try{
                try{
                    faceIndices = plyIn.getElement("face").getListProperty<int>("vertex_index");
                }catch(const std::runtime_error &e1){
                    faceIndices = plyIn.getElement("face").getListProperty<int>("vertex_indices");
                }
            }catch(const std::runtime_error &e1){
                try{
                    unsigned_faceIndices = plyIn.getElement("face").getListProperty<unsigned int>("vertex_index");
                }catch(const std::runtime_error &e1){
                    unsigned_faceIndices = plyIn.getElement("face").getListProperty<unsigned int>("vertex_indices");
                }
            }



            // Fill the fInd structure
            for (const auto &face : faceIndices) {
                fInd.push_back(face);
            }

            std::cout<<"find size: "<<fInd.size()<<std::endl;
            if(faceIndices.size() == 0){
                for (const auto &face : unsigned_faceIndices) {
                    std::vector<int> int_vec;
                    for(auto vert: face){
                        int_vec.push_back((int)vert);
                    }
                    fInd.push_back(int_vec);
                }
            }


            //*****************************************************************************************

            mesh.texture_offset = tex_coord_size - prev_vertex_count;

            for(auto &face:fInd){
                if(face.size() == 3){
                    Face new_face{static_cast<int>(face[0]) + prev_vertex_count+1, static_cast<int>(face[1]) + prev_vertex_count+1,static_cast<int>(face[2]) + prev_vertex_count+1};

                    mesh.faces.push_back(new_face);



                }else if(face.size() == 4){
                    Face new_face{static_cast<int>(face[0]) + prev_vertex_count+1, static_cast<int>(face[1]) + prev_vertex_count+1,static_cast<int>(face[2]) + prev_vertex_count+1};
                    mesh.faces.push_back(new_face);


                    Face new_face2{static_cast<int>(face[0]) + prev_vertex_count+1, static_cast<int>(face[2]) + prev_vertex_count+1,static_cast<int>(face[3]) + prev_vertex_count+1};
                    mesh.faces.push_back(new_face2);

                }


            }


        }
        istream.clear();
        istream.seekg(0, std::ios::beg);

        child = element->FirstChildElement("MotionBlur");
        if(child)
        {
            istream.str(child->GetText());
            istream >> mesh.motion_blur.x >> mesh.motion_blur.y >> mesh.motion_blur.z;
            std::cout<<"read mb"<<std::endl;
            std::cout<<mesh.motion_blur.x<<" "<<mesh.motion_blur.y<<" "<<mesh.motion_blur.z<<std::endl;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            mesh.motion_blur.x = 0;
            mesh.motion_blur.y = 0;
            mesh.motion_blur.z = 0;
        }

        meshes.push_back(mesh);
        mesh.faces.clear();
        istream.clear();
        istream.seekg(0, std::ios::beg);
        element = element->NextSiblingElement("Mesh");
    }
    istream.clear();
    istream.seekg(0, std::ios::beg);


    //Get Meshes
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("CloudMesh");
    while (element)
    {

        mesh.is_light = false;
        mesh.is_cloud = true;
        mesh.id = atoi(element->Attribute("id"));
        mesh.texture_ids.clear();
        mesh.transformations.clear();
        child = element->FirstChildElement("Material");
        istream.str(child->GetText());
        istream >> mesh.material_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        mesh.radiance.x = 0.0;
        mesh.radiance.y = 0.0;
        mesh.radiance.z = 0.0;

        //add texture*******************************************************************************************
        child = element->FirstChildElement("Textures");
        if(child != NULL){
            istream.str(child->GetText());

            std::string texture;

            while(istream >> texture){

                mesh.texture_ids.push_back(stoi(texture));
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }



        //Transformations************************************************************************************
        child = element->FirstChildElement("Transformations");
        if(child){
            istream.str(child->GetText());

            std::string singular_transformation;

            while(istream >> singular_transformation){
                Transformation new_transformation;
                if(singular_transformation.at(0) == 't'){
                    new_transformation.transformation_type = 0;
                }else if(singular_transformation.at(0) == 's'){
                    new_transformation.transformation_type = 1;
                }else if(singular_transformation.at(0) == 'r'){
                    new_transformation.transformation_type = 2;
                }else if(singular_transformation.at(0) == 'c'){
                    new_transformation.transformation_type = 3;
                }

                new_transformation.id = stoi(singular_transformation.substr(1));
                mesh.transformations.push_back(new_transformation);
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }


        //***************************************************************************************************
        child = element->FirstChildElement("Faces");
        bool is_ply = false;
        is_ply = (child->Attribute("plyFile") != NULL);
        if(!is_ply){


            if(child->Attribute("vertexOffset")){
                mesh.vertex_offset = atoi(child->Attribute("vertexOffset"));


            }else{
                mesh.vertex_offset = 0;
            }

            if(child->Attribute("textureOffset")){
                std::cout<<"make bigger"<<std::endl;
                mesh.texture_offset = atoi(child->Attribute("textureOffset"));

            }else{
                std::cout<<"make 0"<<std::endl;
                mesh.texture_offset = 0;
            }



            istream.str(child->GetText());

            Face face;
            while (!(istream >> face.v0_id).eof())
            {
                istream >> face.v1_id >> face.v2_id;
                mesh.faces.push_back(face);


            }
        }else{
            std::cout<<"reading ply"<<std::endl;
            int vertex_offset = 0;
            int texture_offset = 0;
            mesh.vertex_offset = vertex_offset;
            mesh.texture_offset = texture_offset;

            int prev_vertex_count = vertex_data.size();
            std::filesystem::path xmlDir = std::filesystem::path(filepath).parent_path();
            const char* ply_file_route = child->Attribute("plyFile");
            std::filesystem::path fullPlyPath = xmlDir / ply_file_route;
            std::string full_path = fullPlyPath;

            char* c_full_path = new char[full_path.size() + 1];
            strcpy(c_full_path, full_path.c_str());

            happly::PLYData plyIn(c_full_path);

            std::vector<std::string> property_names = plyIn.getElement("vertex").getPropertyNames();

            int tex_coord_size = tex_coord_data.size();

            if(property_names.size() == 3 || property_names.size() == 6){
                std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();


                for(auto &vertex:vPos){
                    Vec3f new_vertex{static_cast<float>(vertex[0]), static_cast<float>(vertex[1]),static_cast<float>(vertex[2])};

                    vertex_data.push_back(new_vertex);
                }
            }else if(property_names.size() == 5){

                std::vector<std::array<double, 5>> vPos = plyIn.getVertexPositionsUV();

                for(auto &vertex:vPos){
                    Vec3f new_vertex{static_cast<float>(vertex[0]), static_cast<float>(vertex[1]),static_cast<float>(vertex[2])};
                    Vec2f new_tex_coord{static_cast<float>(vertex[3]), static_cast<float>(vertex[4])};

                    vertex_data.push_back(new_vertex);
                    tex_coord_data.push_back(new_tex_coord);
                }

            }else if(property_names.size() == 8){
                std::vector<std::array<double, 8>> vPos = plyIn.getVertexPositions8();

                for(auto &vertex:vPos){
                    Vec3f new_vertex{static_cast<float>(vertex[0]), static_cast<float>(vertex[1]),static_cast<float>(vertex[2])};
                    Vec2f new_tex_coord{static_cast<float>(vertex[6]), static_cast<float>(vertex[7])};

                    vertex_data.push_back(new_vertex);
                    tex_coord_data.push_back(new_tex_coord);
                }
            }
            std::cout<<"vertex count: "<<vertex_data.size()<<std::endl;


            //*****************************************************************************************
            std::vector<std::vector<int>> fInd;

            // Read face indices
            std::vector<std::vector<int>> faceIndices;
            std::vector<std::vector<unsigned int>> unsigned_faceIndices;

            try{
                try{
                    faceIndices = plyIn.getElement("face").getListProperty<int>("vertex_index");
                }catch(const std::runtime_error &e1){
                    faceIndices = plyIn.getElement("face").getListProperty<int>("vertex_indices");
                }
            }catch(const std::runtime_error &e1){
                try{
                    unsigned_faceIndices = plyIn.getElement("face").getListProperty<unsigned int>("vertex_index");
                }catch(const std::runtime_error &e1){
                    unsigned_faceIndices = plyIn.getElement("face").getListProperty<unsigned int>("vertex_indices");
                }
            }



            // Fill the fInd structure
            for (const auto &face : faceIndices) {
                fInd.push_back(face);
            }

            std::cout<<"find size: "<<fInd.size()<<std::endl;
            if(faceIndices.size() == 0){
                for (const auto &face : unsigned_faceIndices) {
                    std::vector<int> int_vec;
                    for(auto vert: face){
                        int_vec.push_back((int)vert);
                    }
                    fInd.push_back(int_vec);
                }
            }


            //*****************************************************************************************

            mesh.texture_offset = tex_coord_size - prev_vertex_count;

            for(auto &face:fInd){
                if(face.size() == 3){
                    Face new_face{static_cast<int>(face[0]) + prev_vertex_count+1, static_cast<int>(face[1]) + prev_vertex_count+1,static_cast<int>(face[2]) + prev_vertex_count+1};

                    mesh.faces.push_back(new_face);



                }else if(face.size() == 4){
                    Face new_face{static_cast<int>(face[0]) + prev_vertex_count+1, static_cast<int>(face[1]) + prev_vertex_count+1,static_cast<int>(face[2]) + prev_vertex_count+1};
                    mesh.faces.push_back(new_face);


                    Face new_face2{static_cast<int>(face[0]) + prev_vertex_count+1, static_cast<int>(face[2]) + prev_vertex_count+1,static_cast<int>(face[3]) + prev_vertex_count+1};
                    mesh.faces.push_back(new_face2);

                }


            }


        }
        istream.clear();
        istream.seekg(0, std::ios::beg);

        child = element->FirstChildElement("MotionBlur");
        if(child)
        {
            istream.str(child->GetText());
            istream >> mesh.motion_blur.x >> mesh.motion_blur.y >> mesh.motion_blur.z;
            std::cout<<"read mb"<<std::endl;
            std::cout<<mesh.motion_blur.x<<" "<<mesh.motion_blur.y<<" "<<mesh.motion_blur.z<<std::endl;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            mesh.motion_blur.x = 0;
            mesh.motion_blur.y = 0;
            mesh.motion_blur.z = 0;
        }

        meshes.push_back(mesh);
        mesh.faces.clear();
        istream.clear();
        istream.seekg(0, std::ios::beg);
        element = element->NextSiblingElement("CloudMesh");
    }
    istream.clear();
    istream.seekg(0, std::ios::beg);



    //Get Light Meshes
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("LightMesh");
    while (element)
    {
        std::cout<<"light mesh parsing"<<std::endl;
        mesh.is_cloud = false;
        mesh.is_light = true;
        mesh.id = atoi(element->Attribute("id"));
        mesh.texture_ids.clear();
        mesh.transformations.clear();
        child = element->FirstChildElement("Material");
        istream.str(child->GetText());
        istream >> mesh.material_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        //add texture*******************************************************************************************
        child = element->FirstChildElement("Textures");
        if(child != NULL){
            istream.str(child->GetText());

            std::string texture;

            while(istream >> texture){

                mesh.texture_ids.push_back(stoi(texture));
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }

        //add radiance*******************************************************************************************
        child = element->FirstChildElement("Radiance");
        istream.str(child->GetText());
        istream >> mesh.radiance.x >> mesh.radiance.y >> mesh.radiance.z;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        //Transformations************************************************************************************
        child = element->FirstChildElement("Transformations");
        if(child){
            istream.str(child->GetText());

            std::string singular_transformation;

            while(istream >> singular_transformation){
                Transformation new_transformation;
                if(singular_transformation.at(0) == 't'){
                    new_transformation.transformation_type = 0;
                }else if(singular_transformation.at(0) == 's'){
                    new_transformation.transformation_type = 1;
                }else if(singular_transformation.at(0) == 'r'){
                    new_transformation.transformation_type = 2;
                }else if(singular_transformation.at(0) == 'c'){
                    new_transformation.transformation_type = 3;
                }

                new_transformation.id = stoi(singular_transformation.substr(1));
                mesh.transformations.push_back(new_transformation);
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }


        //***************************************************************************************************
        child = element->FirstChildElement("Faces");
        bool is_ply = false;
        is_ply = (child->Attribute("plyFile") != NULL);
        if(!is_ply){


            if(child->Attribute("vertexOffset")){
                mesh.vertex_offset = atoi(child->Attribute("vertexOffset"));


            }else{
                mesh.vertex_offset = 0;
            }

            if(child->Attribute("textureOffset")){
                std::cout<<"make bigger"<<std::endl;
                mesh.texture_offset = atoi(child->Attribute("textureOffset"));

            }else{
                std::cout<<"make 0"<<std::endl;
                mesh.texture_offset = 0;
            }



            istream.str(child->GetText());

            Face face;
            while (!(istream >> face.v0_id).eof())
            {
                istream >> face.v1_id >> face.v2_id;
                mesh.faces.push_back(face);


            }
        }else{
            std::cout<<"reading ply"<<std::endl;
            int vertex_offset = 0;
            int texture_offset = 0;
            mesh.vertex_offset = vertex_offset;
            mesh.texture_offset = texture_offset;

            int prev_vertex_count = vertex_data.size();
            std::filesystem::path xmlDir = std::filesystem::path(filepath).parent_path();
            const char* ply_file_route = child->Attribute("plyFile");
            std::filesystem::path fullPlyPath = xmlDir / ply_file_route;
            std::string full_path = fullPlyPath;

            char* c_full_path = new char[full_path.size() + 1];
            strcpy(c_full_path, full_path.c_str());

            happly::PLYData plyIn(c_full_path);

            std::vector<std::string> property_names = plyIn.getElement("vertex").getPropertyNames();

            int tex_coord_size = tex_coord_data.size();

            if(property_names.size() == 3 || property_names.size() == 6){
                std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();


                for(auto &vertex:vPos){
                    Vec3f new_vertex{static_cast<float>(vertex[0]), static_cast<float>(vertex[1]),static_cast<float>(vertex[2])};

                    vertex_data.push_back(new_vertex);
                }
            }else if(property_names.size() == 5){

                std::vector<std::array<double, 5>> vPos = plyIn.getVertexPositionsUV();

                for(auto &vertex:vPos){
                    Vec3f new_vertex{static_cast<float>(vertex[0]), static_cast<float>(vertex[1]),static_cast<float>(vertex[2])};
                    Vec2f new_tex_coord{static_cast<float>(vertex[3]), static_cast<float>(vertex[4])};

                    vertex_data.push_back(new_vertex);
                    tex_coord_data.push_back(new_tex_coord);
                }

            }else if(property_names.size() == 8){
                std::vector<std::array<double, 8>> vPos = plyIn.getVertexPositions8();

                for(auto &vertex:vPos){
                    Vec3f new_vertex{static_cast<float>(vertex[0]), static_cast<float>(vertex[1]),static_cast<float>(vertex[2])};
                    Vec2f new_tex_coord{static_cast<float>(vertex[6]), static_cast<float>(vertex[7])};

                    vertex_data.push_back(new_vertex);
                    tex_coord_data.push_back(new_tex_coord);
                }
            }
            std::cout<<"vertex count: "<<vertex_data.size()<<std::endl;


            //*****************************************************************************************
            std::vector<std::vector<int>> fInd;

            // Read face indices
            std::vector<std::vector<int>> faceIndices;
            std::vector<std::vector<unsigned int>> unsigned_faceIndices;

            try{
                try{
                    faceIndices = plyIn.getElement("face").getListProperty<int>("vertex_index");
                }catch(const std::runtime_error &e1){
                    faceIndices = plyIn.getElement("face").getListProperty<int>("vertex_indices");
                }
            }catch(const std::runtime_error &e1){
                try{
                    unsigned_faceIndices = plyIn.getElement("face").getListProperty<unsigned int>("vertex_index");
                }catch(const std::runtime_error &e1){
                    unsigned_faceIndices = plyIn.getElement("face").getListProperty<unsigned int>("vertex_indices");
                }
            }



            // Fill the fInd structure
            for (const auto &face : faceIndices) {
                fInd.push_back(face);
            }

            std::cout<<"find size: "<<fInd.size()<<std::endl;
            if(faceIndices.size() == 0){
                for (const auto &face : unsigned_faceIndices) {
                    std::vector<int> int_vec;
                    for(auto vert: face){
                        int_vec.push_back((int)vert);
                    }
                    fInd.push_back(int_vec);
                }
            }


            //*****************************************************************************************

            mesh.texture_offset = tex_coord_size - prev_vertex_count;

            for(auto &face:fInd){
                if(face.size() == 3){
                    Face new_face{static_cast<int>(face[0]) + prev_vertex_count+1, static_cast<int>(face[1]) + prev_vertex_count+1,static_cast<int>(face[2]) + prev_vertex_count+1};

                    mesh.faces.push_back(new_face);



                }else if(face.size() == 4){
                    Face new_face{static_cast<int>(face[0]) + prev_vertex_count+1, static_cast<int>(face[1]) + prev_vertex_count+1,static_cast<int>(face[2]) + prev_vertex_count+1};
                    mesh.faces.push_back(new_face);


                    Face new_face2{static_cast<int>(face[0]) + prev_vertex_count+1, static_cast<int>(face[2]) + prev_vertex_count+1,static_cast<int>(face[3]) + prev_vertex_count+1};
                    mesh.faces.push_back(new_face2);

                }


            }


        }
        istream.clear();
        istream.seekg(0, std::ios::beg);

        child = element->FirstChildElement("MotionBlur");
        if(child)
        {
            istream.str(child->GetText());
            istream >> mesh.motion_blur.x >> mesh.motion_blur.y >> mesh.motion_blur.z;
            std::cout<<"read mb"<<std::endl;
            std::cout<<mesh.motion_blur.x<<" "<<mesh.motion_blur.y<<" "<<mesh.motion_blur.z<<std::endl;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            mesh.motion_blur.x = 0;
            mesh.motion_blur.y = 0;
            mesh.motion_blur.z = 0;
        }

        meshes.push_back(mesh);
        mesh.faces.clear();
        istream.clear();
        istream.seekg(0, std::ios::beg);
        element = element->NextSiblingElement("LightMesh");
    }
    istream.clear();
    istream.seekg(0, std::ios::beg);




    //Get Mesh Instances
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("MeshInstance");
    MeshInstance mesh_instance;
    while (element)
    {


        mesh_instance.corresponding_mesh_id = atoi(element->Attribute("baseMeshId"));
        mesh_instance.id = atoi(element->Attribute("id"));

        if(element->Attribute("resetTransform")){

            if(strcmp(element->Attribute("resetTransform"), "true") == 0){
                mesh_instance.reset_transform = true;
            }else{
                mesh_instance.reset_transform = false;
            }
        }else{
            mesh_instance.reset_transform = false;
        }

        mesh_instance.texture_ids.clear();
        mesh_instance.transformations.clear();
        child = element->FirstChildElement("Material");
        if (child){
            mesh_instance.is_new_material = true;
            istream.str(child->GetText());
            istream >> mesh_instance.material_id;
        }else{
            mesh_instance.is_new_material = false;
            mesh_instance.material_id = -1;
        }

        istream.clear();
        istream.seekg(0, std::ios::beg);

        //radiance**********************************************************************************************
        child = element->FirstChildElement("Radiance");

        if(child){
            mesh_instance.is_light = true;
            istream.str(child->GetText());
            istream >> mesh_instance.radiance.x >> mesh_instance.radiance.y >> mesh_instance.radiance.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            mesh_instance.is_light = false;
            mesh_instance.radiance.x = 0.0;
            mesh_instance.radiance.y = 0.0;
            mesh_instance.radiance.z = 0.0;
        }



        //Transformations************************************************************************************
        child = element->FirstChildElement("Transformations");
        if(child){
            istream.str(child->GetText());

            std::string singular_transformation;

            while(istream >> singular_transformation){
                Transformation new_transformation;
                if(singular_transformation.at(0) == 't'){
                    new_transformation.transformation_type = 0;
                }else if(singular_transformation.at(0) == 's'){
                    new_transformation.transformation_type = 1;
                }else if(singular_transformation.at(0) == 'r'){
                    new_transformation.transformation_type = 2;
                }else if(singular_transformation.at(0) == 'c'){
                    new_transformation.transformation_type = 3;
                }

                new_transformation.id = stoi(singular_transformation.substr(1));
                mesh_instance.transformations.push_back(new_transformation);
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }
        child = element->FirstChildElement("MotionBlur");
        if(child)
        {
            istream.str(child->GetText());
            istream >> mesh_instance.motion_blur.x >> mesh_instance.motion_blur.y >> mesh_instance.motion_blur.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            mesh_instance.motion_blur.x = 0;
            mesh_instance.motion_blur.y = 0;
            mesh_instance.motion_blur.z = 0;
        }

        mesh_instances.push_back(mesh_instance);
        istream.clear();
        istream.seekg(0, std::ios::beg);
        element = element->NextSiblingElement("MeshInstance");
    }
    istream.clear();
    istream.seekg(0, std::ios::beg);



    //TRIANGLES**********************************************************************************************
    //Get Triangles
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Triangle");
    Triangle triangle;
    while (element)
    {
        triangle.is_light = false;
        triangle.texture_ids.clear();
        triangle.transformations.clear();
        child = element->FirstChildElement("Material");
        istream.str(child->GetText());
        istream >> triangle.material_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        triangle.radiance.x = 0.0;
        triangle.radiance.y = 0.0;
        triangle.radiance.z = 0.0;

        //Textures********************************************************************************************
        child = element->FirstChildElement("Textures");
        if(child){
            istream.str(child->GetText());

            std::string texture;

            while(istream >> texture){

                triangle.texture_ids.push_back(stoi(texture));
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }

        //Transformations************************************************************************************
        child = element->FirstChildElement("Transformations");
        if(child){
            istream.str(child->GetText());

            std::string singular_transformation;

            while(istream >> singular_transformation){
                Transformation new_transformation;
                if(singular_transformation.at(0) == 't'){
                    new_transformation.transformation_type = 0;
                }else if(singular_transformation.at(0) == 's'){
                    new_transformation.transformation_type = 1;
                }else if(singular_transformation.at(0) == 'r'){
                    new_transformation.transformation_type = 2;
                }else if(singular_transformation.at(0) == 'c'){
                    new_transformation.transformation_type = 3;
                }

                new_transformation.id = stoi(singular_transformation.substr(1));
                triangle.transformations.push_back(new_transformation);
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }

        //***************************************************************************************************

        child = element->FirstChildElement("Indices");
        istream.str(child->GetText());
        istream >> triangle.indices.v0_id >> triangle.indices.v1_id >> triangle.indices.v2_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        child = element->FirstChildElement("MotionBlur");
        if(child)
        {
            istream.str(child->GetText());
            istream >> triangle.motion_blur.x >> triangle.motion_blur.y >> triangle.motion_blur.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            triangle.motion_blur.x = 0;
            triangle.motion_blur.y = 0;
            triangle.motion_blur.z = 0;
        }

        triangles.push_back(triangle);
        element = element->NextSiblingElement("Triangle");
    }


    //Get Triangle Lights
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("LightTriangle");
    while (element)
    {
        triangle.is_light = true;
        triangle.texture_ids.clear();
        triangle.transformations.clear();
        child = element->FirstChildElement("Material");
        istream.str(child->GetText());
        istream >> triangle.material_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        //radiance**********************************************************************************************
        child = element->FirstChildElement("Radiance");
        istream.str(child->GetText());
        istream >> triangle.radiance.x >> triangle.radiance.y >> triangle.radiance.z;
        istream.clear();
        istream.seekg(0, std::ios::beg);


        //Textures********************************************************************************************
        child = element->FirstChildElement("Textures");
        if(child){
            istream.str(child->GetText());

            std::string texture;

            while(istream >> texture){

                triangle.texture_ids.push_back(stoi(texture));
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }

        //Transformations************************************************************************************
        child = element->FirstChildElement("Transformations");
        if(child){
            istream.str(child->GetText());

            std::string singular_transformation;

            while(istream >> singular_transformation){
                Transformation new_transformation;
                if(singular_transformation.at(0) == 't'){
                    new_transformation.transformation_type = 0;
                }else if(singular_transformation.at(0) == 's'){
                    new_transformation.transformation_type = 1;
                }else if(singular_transformation.at(0) == 'r'){
                    new_transformation.transformation_type = 2;
                }else if(singular_transformation.at(0) == 'c'){
                    new_transformation.transformation_type = 3;
                }

                new_transformation.id = stoi(singular_transformation.substr(1));
                triangle.transformations.push_back(new_transformation);
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }


        //***************************************************************************************************

        child = element->FirstChildElement("Indices");
        istream.str(child->GetText());
        istream >> triangle.indices.v0_id >> triangle.indices.v1_id >> triangle.indices.v2_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        child = element->FirstChildElement("MotionBlur");
        if(child)
        {
            istream.str(child->GetText());
            istream >> triangle.motion_blur.x >> triangle.motion_blur.y >> triangle.motion_blur.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            triangle.motion_blur.x = 0;
            triangle.motion_blur.y = 0;
            triangle.motion_blur.z = 0;
        }

        triangles.push_back(triangle);
        element = element->NextSiblingElement("Triangle");
    }



    //SPHERES**********************************************************************************************
    //Get Spheres
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Sphere");
    Sphere sphere;
    while (element)
    {
        sphere.texture_ids.clear();
        sphere.transformations.clear();
        sphere.is_light = false;
        sphere.is_cloud = false;
        child = element->FirstChildElement("Material");
        istream.str(child->GetText());
        istream >> sphere.material_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        sphere.radiance.x = 0.0;
        sphere.radiance.y = 0.0;
        sphere.radiance.z = 0.0;

        //add texture*******************************************************************************************
        child = element->FirstChildElement("Textures");
        if(child){
            istream.str(child->GetText());

            std::string texture;

            while(istream >> texture){

                sphere.texture_ids.push_back(stoi(texture));
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }

        //Transformations************************************************************************************
        child = element->FirstChildElement("Transformations");
        if(child){
            istream.str(child->GetText());

            std::string singular_transformation;
            while(istream >> singular_transformation){
                Transformation new_transformation;
                if(singular_transformation.at(0) == 't'){
                    new_transformation.transformation_type = 0;
                }else if(singular_transformation.at(0) == 's'){
                    new_transformation.transformation_type = 1;
                }else if(singular_transformation.at(0) == 'r'){
                    new_transformation.transformation_type = 2;
                }else if(singular_transformation.at(0) == 'c'){
                    new_transformation.transformation_type = 3;
                }

                new_transformation.id = stoi(singular_transformation.substr(1));
                sphere.transformations.push_back(new_transformation);
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }


        //***************************************************************************************************


        child = element->FirstChildElement("Center");
        istream.str(child->GetText());
        istream >> sphere.center_vertex_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);



        child = element->FirstChildElement("Radius");
        istream.str(child->GetText());
        istream >> sphere.radius;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        child = element->FirstChildElement("MotionBlur");
        if(child)
        {
            istream.str(child->GetText());
            istream >> sphere.motion_blur.x >> sphere.motion_blur.y >> sphere.motion_blur.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            sphere.motion_blur.x = 0;
            sphere.motion_blur.y = 0;
            sphere.motion_blur.z = 0;
        }

        spheres.push_back(sphere);
        element = element->NextSiblingElement("Sphere");
    }


    //Get Sphere Lights
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("LightSphere");
    while (element)
    {
        sphere.texture_ids.clear();
        sphere.is_light = true;
        sphere.is_cloud = false;
        sphere.transformations.clear();
        child = element->FirstChildElement("Material");
        istream.str(child->GetText());
        istream >> sphere.material_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);



        //radiance**********************************************************************************************
        child = element->FirstChildElement("Radiance");
        istream.str(child->GetText());
        istream >> sphere.radiance.x >> sphere.radiance.y >> sphere.radiance.z;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        //add texture*******************************************************************************************
        child = element->FirstChildElement("Textures");
        if(child){
            istream.str(child->GetText());

            std::string texture;

            while(istream >> texture){

                sphere.texture_ids.push_back(stoi(texture));
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }

        //Transformations************************************************************************************
        child = element->FirstChildElement("Transformations");
        if(child){
            istream.str(child->GetText());

            std::string singular_transformation;
            while(istream >> singular_transformation){
                Transformation new_transformation;
                if(singular_transformation.at(0) == 't'){
                    new_transformation.transformation_type = 0;
                }else if(singular_transformation.at(0) == 's'){
                    new_transformation.transformation_type = 1;
                }else if(singular_transformation.at(0) == 'r'){
                    new_transformation.transformation_type = 2;
                }else if(singular_transformation.at(0) == 'c'){
                    new_transformation.transformation_type = 3;
                }

                new_transformation.id = stoi(singular_transformation.substr(1));
                sphere.transformations.push_back(new_transformation);
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }


        //***************************************************************************************************


        child = element->FirstChildElement("Center");
        istream.str(child->GetText());
        istream >> sphere.center_vertex_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);



        child = element->FirstChildElement("Radius");
        istream.str(child->GetText());
        istream >> sphere.radius;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        child = element->FirstChildElement("MotionBlur");
        if(child)
        {
            istream.str(child->GetText());
            istream >> sphere.motion_blur.x >> sphere.motion_blur.y >> sphere.motion_blur.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            sphere.motion_blur.x = 0;
            sphere.motion_blur.y = 0;
            sphere.motion_blur.z = 0;
        }

        spheres.push_back(sphere);
        element = element->NextSiblingElement("LightSphere");
    }


    //Get Sphere Clouds
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("CloudSphere");
    while (element)
    {
        sphere.texture_ids.clear();
        sphere.is_light = false;
        sphere.is_cloud = true;
        sphere.transformations.clear();
        child = element->FirstChildElement("Material");
        istream.str(child->GetText());
        istream >> sphere.material_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);



        //add texture*******************************************************************************************
        child = element->FirstChildElement("Textures");
        if(child){
            istream.str(child->GetText());

            std::string texture;

            while(istream >> texture){

                sphere.texture_ids.push_back(stoi(texture));
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }

        //Transformations************************************************************************************
        child = element->FirstChildElement("Transformations");
        if(child){
            istream.str(child->GetText());

            std::string singular_transformation;
            while(istream >> singular_transformation){
                Transformation new_transformation;
                if(singular_transformation.at(0) == 't'){
                    new_transformation.transformation_type = 0;
                }else if(singular_transformation.at(0) == 's'){
                    new_transformation.transformation_type = 1;
                }else if(singular_transformation.at(0) == 'r'){
                    new_transformation.transformation_type = 2;
                }else if(singular_transformation.at(0) == 'c'){
                    new_transformation.transformation_type = 3;
                }

                new_transformation.id = stoi(singular_transformation.substr(1));
                sphere.transformations.push_back(new_transformation);
            }


            istream.clear();
            istream.seekg(0, std::ios::beg);

        }


        //***************************************************************************************************


        child = element->FirstChildElement("Center");
        istream.str(child->GetText());
        istream >> sphere.center_vertex_id;
        istream.clear();
        istream.seekg(0, std::ios::beg);



        child = element->FirstChildElement("Radius");
        istream.str(child->GetText());
        istream >> sphere.radius;
        istream.clear();
        istream.seekg(0, std::ios::beg);

        child = element->FirstChildElement("MotionBlur");
        if(child)
        {
            istream.str(child->GetText());
            istream >> sphere.motion_blur.x >> sphere.motion_blur.y >> sphere.motion_blur.z;
            istream.clear();
            istream.seekg(0, std::ios::beg);
        }else{
            sphere.motion_blur.x = 0;
            sphere.motion_blur.y = 0;
            sphere.motion_blur.z = 0;
        }

        spheres.push_back(sphere);
        element = element->NextSiblingElement("CloudSphere");
    }


}
