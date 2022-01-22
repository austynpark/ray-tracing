#include "acceleration.h"


/////////////////////////////
// Vector and ray conversions
bvh::Vector3<float> vec3ToBvh(const vec3& v)
{
    return bvh::Vector3<float>(v[0], v[1], v[2]);
}

vec3 vec3FromBvh(const bvh::Vector3<float>& v)
{
    return vec3(v[0], v[1], v[2]);
}

bvh::Ray<float> RayToBvh(const Ray& r)
{
    // Construct a bvh::Ray<float> with values
    // r's origin, r's direction, 0, INFINIFY
}

Ray RayFromBvh(const bvh::Ray<float>& r)
{
    // Construct one of your rays from r.origin, and r.direction
}


/////////////////////////////
// SimpleBox
SimpleBox::SimpleBox() : bvh::BoundingBox<float>() {}
SimpleBox::SimpleBox(const vec3 v) : bvh::BoundingBox<float>(vec3ToBvh(v)) {}

SimpleBox& SimpleBox::extend(const vec3 v)
{
    bvh::BoundingBox<float>::extend(vec3ToBvh(v));
    return *this;
}


/////////////////////////////
// BvhShape

SimpleBox BvhShape::bounding_box() const
{
    //  Return the shape's bounding box.
}

bvh::Vector3<float> BvhShape::center() const
{
    return bounding_box().center();
}

std::optional<IntersectionRecord> BvhShape::intersect(const bvh::Ray<float>& bvhray) const
{
    // Intersect RayFromBvh(bvhray) with shape;  store result in an IntersectionRecord
    // If no intersection,
    //    return std::nullopt;
    // If intersection's t value < bvhray.tmin  or > bvhray.tmax
    //    return std::nullopt;
    // else return
    //    return the IntersectionRecord
}

AccelerationBvh::AccelerationBvh(std::vector<Shape*>& objs)
{
    // Wrap all Shape*'s with a bvh specific instance
    for (Shape* shape : objs) {
        shapeVector.emplace_back(shape);
    }

    // Magic found in the bvh examples:
    auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(shapeVector.data(),
        shapeVector.size());
    auto global_bbox = bvh::compute_bounding_boxes_union(bboxes.get(), shapeVector.size());

    bvh::SweepSahBuilder<bvh::Bvh<float>> builder(bvh);
    builder.build(global_bbox, bboxes.get(), centers.get(), shapeVector.size());
}

IntersectionRecord AccelerationBvh::intersect(const Ray& ray)
{
    bvh::Ray<float> bvhray = RayToBvh(ray);

    // Magic found in the bvh examples:
    bvh::ClosestPrimitiveIntersector<bvh::Bvh<float>, BvhShape> intersector(bvh, shapeVector.data());
    bvh::SingleRayTraverser<bvh::Bvh<float>> traverser(bvh);

    auto hit = traverser.traverse(bvhray, intersector);
    if (hit) {
        return hit->intersection;;
    }
    else
        //return  NO-INTERSECTION


}