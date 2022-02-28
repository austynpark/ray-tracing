//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
	#include <chrono>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"
#include "camera.h"
#include "shape.h"
#include "acceleration.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
constexpr float RussianRoulette = 0.8f;
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<double> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"
void WriteHdrImage(const std::string outName, const int width, const int height, Color* image, int pass)
{
    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
    float* data = new float[width*height*3];
    float* dp = data;
    for (int y=height-1;  y>=0;  --y) {
        for (int x=0;  x<width;  ++x) {
            Color pixel = image[y*width + x] / (float)pass;
            *dp++ = pixel[0];
            *dp++ = pixel[1];
            *dp++ = pixel[2]; } }

    // Write image to file in HDR (a.k.a RADIANCE) format
    rgbe_header_info info;
    char errbuf[100] = {0};

    FILE* fp  =  fopen(outName.c_str(), "wb");
    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    r = RGBE_WritePixels_RLE(fp, data, width,  height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);
    
    delete data;
}


Scene::Scene() 
{ 
    camera = std::make_unique<Camera>();
}

Scene::~Scene()
{
    for (auto& shape : shapes) {
        if (shape) {
            delete shape;
            shape = 0;
        }
    }

    if (currentMat) {
        delete currentMat;
        currentMat = 0;
    }

}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{
    for (const auto& triangle_index : mesh->triangles) {
        shapes.push_back(new Triangle(
            mesh->vertices[triangle_index[0]].pnt,
            mesh->vertices[triangle_index[1]].pnt,
            mesh->vertices[triangle_index[2]].pnt,
            mesh->vertices[triangle_index[0]].nrm,
            mesh->vertices[triangle_index[1]].nrm,
            mesh->vertices[triangle_index[2]].nrm,
            mesh->mat
        ));
    }
}

quat Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    quat q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Xaxis());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Yaxis());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Zaxis());
        else if (c == "q")  {
            q *= quat(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, normalize(vec3(f[i+1], f[i+2], f[i+3])));
            i+=4; } }
    return q;
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        // syntax: screen width height
        width = int(f[1]);
        height = int(f[2]); }

    else if (c == "camera") {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        //realtime->setCamera(vec3(f[1],f[2],f[3]), Orientation(5,strings,f), f[4]);
        camera->E = vec3(f[1], f[2], f[3]);
        camera->Q = Orientation(5, strings, f);
        camera->ry = f[4];
    }

    else if (c == "ambient") {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        //realtime->setAmbient(vec3(f[1], f[2], f[3]));
    }

    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7]);
    }

    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(vec3(f[1], f[2], f[3])); 
    }
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        // realtime->sphere(vec3(f[1], f[2], f[3]), f[4], currentMat);
        shapes.push_back(new Sphere(vec3(f[1], f[2], f[3]), f[4], currentMat));

        if (currentMat->isLight()) {
            lights.push_back(shapes.back());
        }
    }
    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        // realtime->box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat);
        shapes.push_back(new Box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat));

        if (currentMat->isLight()) {
            lights.push_back(shapes.back());
        }
    }
    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        // realtime->cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat);
        shapes.push_back(new Cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat));

        if (currentMat->isLight()) {
            lights.push_back(shapes.back());
        }
    }


    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        mat4 modelTr = translate(vec3(f[2],f[3],f[4]))
                          *scale(vec3(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);  
    
        //Triangle(vec3 V0, vec3 V1, vec3 V2, vec3 N0, vec3 N1, vec3 N2, Material * mat);    
    }

    
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

void Scene::TraceImage(Color* image, const unsigned int max_pass)
{
 
    float rx = (camera->ry * width) / height;

    vec3 vecX_world_space = rx * transformVector(camera->Q, Xaxis());
    vec3 vecY_world_space = camera->ry * transformVector(camera->Q, Yaxis());
    vec3 vecZ_world_space = transformVector(camera->Q, Zaxis());

    int shapes_size = shapes.size();

    bvh_data = std::make_unique<AccelerationBvh>(shapes);

    auto startTime = std::chrono::high_resolution_clock::now();

    for (unsigned int pass = 0; pass < max_pass; ++pass) {
#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y = 0; y < height; ++y) {

            fprintf(stderr, "Rendering %4d\r", y);
            for (int x = 0; x < width; ++x) {

                // x, y to [-1, 1] sceen space
                float dx = 2 * (x + myrandom(RNGen)) / width - 1;
                float dy = 2 * (y + myrandom(RNGen)) / height - 1;

                // genenrate ray from camera to screen position
                Ray ray{ camera->E, normalize(dx * vecX_world_space + dy * vecY_world_space - vecZ_world_space) };


                Color color{};




                //image[y * width + x] = glm::abs(intersect.N);
                //image[y * width + x] = vec3((intersect.t - 5) / 4);
                //image[y * width + x] = intersect.object->material->Kd;
                image[y * width + x] += TracePath(&ray);
            }
        }

        if (((pass + 1) & pass) == 0) {

            std::string hdrName ="output_" + std::to_string(pass + 1) +".hdr";
            WriteHdrImage(hdrName, width, height, image,  (pass + 1));
        }

    }
    fprintf(stderr, "\n");


    auto endTime = std::chrono::high_resolution_clock::now();

    double timeDuration = std::chrono::duration< double, std::milli >(endTime - startTime).count();

    std::cout << "OBJ file read in "
        << timeDuration
        << "  milli seconds." << std::endl;

}

vec3 Scene::TracePath(Ray* pRay)
{
    Color color{ 0, 0, 0 }; // Accumulated light
    vec3 weight{ 1, 1, 1 }; // Accumulated weight

    Intersection intersect = bvh_data->intersect(*pRay);

    if (intersect.object == nullptr) {
        return color;
    }

    if (intersect.object->material->isLight()) {
        return EvalRadiance(intersect);
    }

    while (myrandom(RNGen) <= RussianRoulette) {

        //Explicit light connection
        Intersection L = SampleLight(); // Randomly choose a light and a point on that light.
        float p = PdfLight(L) / GeometryFactor(intersect, L); // Probability of L, converted to angular measure
        
        // Input dir for next shape & out dir for the current shape
        vec3 input_dir = L.P - intersect.P;
        Ray shadow_ray{ intersect.P,input_dir }; //Trace ray from P toward L (called a shadow-ray)
        
        Intersection I = bvh_data->intersect(shadow_ray);
        
        if (p > 0 && (I.object == L.object)) {
            vec3 f = EvalScattering(intersect, input_dir);
            color += 0.5f * weight * (f / p) * EvalRadiance(L);
        }
    

        //Extend path
        input_dir = SampleBrdf(intersect.N); // Choose a sample direction from P
        Ray Q{ intersect.P, input_dir };

        I = bvh_data->intersect(Q);

        if (I.object == nullptr) {
            break;
        }

        vec3 f = EvalScattering(intersect, input_dir);
        p = PdfBrdf(intersect.N, input_dir) * RussianRoulette;

        if (p < 0.000001f) break; // Avoid division by zero or nearly zero
        weight *= f / p;

        if (I.object->material->isLight()) {
            color += weight * EvalRadiance(I);
            break;
        }

        intersect = I;
    }

/*
    if (intersect.object) {
        if (!intersect.object->material->isLight()) {
            for (const auto& light : lights) {
                color += intersect.object->material->Kd * dot(intersect.N, normalize(light->position - intersect.P)) + intersect.object->material->Ks;
            }
        }
        else {
            color = intersect.object->material->Kd;
        }
    }
*/

    return color;
}

vec3 Scene::EvalRadiance(const Intersection& intersect)
{
    return intersect.object->material->Kd / PI;
}

Intersection Scene::SampleSphere(Sphere* sphere)
{
    const float z = 2 * myrandom(RNGen) - 1;
    const float r = sqrtf(1 - z * z);
    const float a = 2 * PI * myrandom(RNGen);

    Intersection result{};

    result.N = vec3( r * cos(a), r * sin(a), z );
    result.P = sphere->C + sphere->r * result.N;
    result.object = sphere;

    return result;
}

Intersection Scene::SampleLight()
{
    assert(lights.size());

    return SampleSphere((Sphere*)lights[myrandom(RNGen) * lights.size()]);
}

float Scene::PdfLight(const Intersection& intersect)
{
    assert(lights.size());

    return 1 / (intersect.object->area * lights.size());
}

float Scene::GeometryFactor(const Intersection& A, const Intersection& B)
{
    vec3 D = A.P - B.P;

    return glm::abs(dot(A.N, D) * dot(B.N, D) / (dot(D, D) * dot(D, D)));
}

vec3 Scene::SampleLobe(vec3 A, float c, float phi)
{
    float s = sqrtf(1 - c * c);
    // Create vector K centered around Z-axis and rotate to A-axis
    vec3 K(s * cos(phi), s * sin(phi), c); // Vector centered around Z-axis

    if (abs(A.z - 1) < 0.001f) return K; // A = Z so no rotation
    if (abs(A.z + 1) < 0.001f) return { K.x, -K.y, -K.z }; // A = -Z  so rotate 180 around X axis

    assert(glm::isNormalized(A, EPSILON));

    // A = normalize(A)
    vec3 B = normalize(vec3(-A.y, A.x, 0)); // Z x A
    vec3 C = cross(A, B);

    return K.x * B + K.y * C + K.z * A;
}

vec3 Scene::SampleBrdf(vec3 N)
{
    return SampleLobe(N, sqrtf(myrandom(RNGen)), 2 * PI * myrandom(RNGen));
}

float Scene::PdfBrdf(vec3 N, vec3 input_dir)
{
    return abs(dot(N, input_dir));
}

vec3 Scene::EvalScattering(const Intersection& intersect, vec3 input_dir)
{
    return abs(dot(intersect.N, input_dir)) * intersect.object->material->Kd / PI;
}
