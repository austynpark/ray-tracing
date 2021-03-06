///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////
#include <memory>
#include <vector>
#include <string>

#include "ray_util.inl"

class Shape;
struct Camera;

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
 public:
    vec3 Kd, Ks;
    float alpha;
    unsigned int texid;

    virtual bool isLight() { return false; }

    Material()  : Kd(vec3(1.0, 0.5, 0.0)), Ks(vec3(1,1,1)), alpha(1.0), texid(0) {}
    Material(const vec3 d, const vec3 s, const float a) 
        : Kd(d), Ks(s), alpha(a), texid(0) {}
    Material(Material& o) { Kd=o.Kd;  Ks=o.Ks;  alpha=o.alpha;  texid=o.texid; }
    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (ivec3: consisting of three
// indices into the vertex array).
    
class VertexData
{
 public:
    vec3 pnt;
    vec3 nrm;
    vec2 tex;
    vec3 tan;
    VertexData(const vec3& p, const vec3& n, const vec2& t, const vec3& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

struct MeshData
{
    std::vector<VertexData> vertices;
    std::vector<ivec3> triangles;
    Material *mat;
};

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:

    Light(const vec3 e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }
    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////////////
// Scene
class AccelerationBvh;
class Sphere;

class Scene {
public:
    int width, height;
    std::unique_ptr<Camera> camera;
    //Realtime* realtime;         // Remove this (realtime stuff)
    
    Material* currentMat;

    std::vector<Shape*> shapes;
    std::vector<Shape*> lights;
    std::unique_ptr<AccelerationBvh> bvh_data;

    Scene();
    ~Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const mat4& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const unsigned int pass);

    vec3 TracePath(Ray* ray);

	// return the RGB radiance of the light
	vec3 EvalRadiance(const Intersection& intersect);
	Intersection SampleSphere(Sphere* sphere);
	Intersection SampleLight();
    float PdfLight(const Intersection& intersect);

    // Convert between angular measure and area measure
    float GeometryFactor(const Intersection& A, const Intersection& B);

    // Choose a direction vector distributed around a given vector A
    // c : cosine of the angle between the returned vector and A
    // phi : an angle around A
    vec3 SampleLobe(const vec3& A, float c, float phi);

    vec3 SampleBrdf(const vec3& output_dir,
        const Intersection& intersect,
        float prob_diffuse, float prob_reflect);

    float PdfBrdf(const vec3& output_dir, const Intersection& intersect, const vec3& input_dir, float prob_diffuse, float prob_reflect);
    vec3 EvalScattering(const vec3& output_dir, const Intersection& intersect, const vec3& input_dir);
    
    

};

