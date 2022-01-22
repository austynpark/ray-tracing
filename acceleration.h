// This uses a library called bvh for a bounding volume hierarchy acceleration structure.
// See https://github.com/madmann91/bvh

// The bvh library uses its own version of vectors, bounding boxes,
// rays, and list of shapes, so there will be some conversions to/from
// the raytracer's versions of these structures.  Alternatively, the
// raytracer could be written to directly use bvh's version of these
// structures.  This seems feasible, but has not been tested.


#include <bvh/bvh.hpp>
#include <bvh/vector.hpp>
#include <bvh/ray.hpp>
#include <bvh/sweep_sah_builder.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>

// Vectors:
// Expectation: The raytracer uses glm::vec3 throughout 
// These convert between glm::vec3 and bvh::Vector3<float>
bvh::Vector3<float> vec3ToBvh(const vec3& v);
vec3 vec3FromBvh(const bvh::Vector3<float>& v);

// Rays:  Rays have an origin and a direction.  bvh::Ray also has a tmin and a tmax.
// These convert between the two.
bvh::Ray<float> RayToBvh(const Ray& r);
Ray RayFromBvh(const bvh::Ray<float>& r);

// Bounding boxes
// Expectation: The raytracer uses SimpleBox throughout.
// This class derives from the bvh bounding box, overloading two methods to take glm::vec3.
// Use: Construct a BB with zero or one points (vec3), extend it with further points.
//      Access its bounds as two bvh vectors box.min and box.max.
class SimpleBox : public bvh::BoundingBox<float> {
public:
    SimpleBox();
    SimpleBox(const vec3 v);
    SimpleBox& extend(const vec3 v);
};


// Wrapper of a single Shape*:

// The ray tracer stores the scene objects as a list of Shape*.  BVH
// expects a list of NON-POINTERS with various methods and type
// declarations.

class BvhShape {
    Shape* shape;
public:
    using ScalarType = float;   // Float or double for rays and vectors.
    using IntersectionType = IntersectionRecord; // Specify the intersection record type.

    BvhShape(Shape* s) : shape(s) { }; // Constructor given the Shape to wrap

    SimpleBox bounding_box() const; // Returns the bounding box of the shape
    bvh::Vector3<float> center() const; // Returns the bounding_box().center()

    // The intersection routine.
    // Given a bvh::Ray, intersect it with the shape and return either of:
    //     an IntersectionRecord
    //         ( it exists, and is between the ray's tmin and tmax.
    //     std::nullopt ((otherwise)
    std::optional<IntersectionRecord> intersect(const bvh::Ray<float>& bvhray) const;
};

// Encapsulates the BVH structure, the list of shapes it's built from,
// and method to intersect a ray with the full scene and return the
// front most intersection point.
class AccelerationBvh {
    bvh::Bvh<float> bvh;
    std::vector<BvhShape> shapeVector;
public:
    AccelerationBvh(std::vector<Shape*>& objs);
    IntersectionRecord intersect(const Ray& ray);
};
