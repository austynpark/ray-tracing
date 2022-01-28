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
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"
#include "realtime.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

#include "camera.h"
#include "shape.h"

Scene::Scene() 
{ 
    camera = std::make_unique<Camera>();
}

Scene::~Scene()
{
    for (auto& shape : shapes) {
        delete shape;
        shape = nullptr;
    }
}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
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
        //currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7])
    }

    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(vec3(f[1], f[2], f[3])); }
   
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        // realtime->sphere(vec3(f[1], f[2], f[3]), f[4], currentMat);
        shapes.push_back(new Sphere(vec3(f[1], f[2], f[3]), f[4], currentMat));
    }

    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        // realtime->box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat);
        shapes.push_back(new Box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat));
    }

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        // realtime->cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat);
        shapes.push_back(new Cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat));
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
    
    
        //shapes.push_back(new Triangle());
    
    }

    
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

void Scene::TraceImage(Color* image, const int pass)
{
 
    float rx = (camera->ry * width) / height;

    vec3 vecX_world_space = rx * transformVector(camera->Q, Xaxis());
    vec3 vecY_world_space = camera->ry * transformVector(camera->Q, Yaxis());
    vec3 vecZ_world_space = transformVector(camera->Q, Zaxis());

    int shapes_size = shapes.size();

//#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y=0;  y<height;  y++) {

        fprintf(stderr, "Rendering %4d\r", y);
        for (int x = 0; x < width; x++) {

            Intersection out_intersect{FLT_MAX};

            //Color color{};

            float dx = 2 * (x + 0.5f) / width - 1;
            float dy = 2 * (y + 0.5f) / height - 1;

			Ray ray{camera->E, normalize(dx * vecX_world_space + dy * vecY_world_space - vecZ_world_space)};

			Intersection intersect{};

            for (int i = 0; i < shapes_size; ++i) {
                
                if (shapes.at(i)->intersect(ray, intersect)) {
                    if (out_intersect.t > intersect.t) {
                        out_intersect = intersect;
                    }
                }
            }
            
            image[y * width + x] = glm::abs(out_intersect.N);
            //image[y * width + x] = vec3((out_intersect.t - 5) / 4);
            

            

            /*
            if ((x-width/2)*(x-width/2)+(y-height/2)*(y-height/2) < 100*100)
                color = Color(myrandom(RNGen), myrandom(RNGen), myrandom(RNGen));
            else if (abs(x-width/2)<4 || abs(y-height/2)<4)
                color = Color(0.0, 0.0, 0.0);
            else
                color = Color(1.0, 1.0, 1.0);
            */
            //image[y * width + x] = color;
        }
    }
    fprintf(stderr, "\n");
}
