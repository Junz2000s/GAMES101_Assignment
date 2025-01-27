#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    bool USE_SAH = true;
    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else 
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        ulong size1;

        if (USE_SAH){
            std::vector<double> surface_areas_a, surface_areas_b;
            double sa = 0, sb = 0, cost = std::numeric_limits<double>::max();
            Bounds3 bound_curr;
            for (auto obj: objects) {
                bound_curr = Union(bound_curr, obj->getBounds());
                surface_areas_a.push_back(bound_curr.SurfaceArea());
            }
            Bounds3 bound_curr_1;
            for (auto obj = objects.end() - 1; obj >= objects.begin(); obj--){
                bound_curr_1 = Union(bound_curr_1, (*obj)->getBounds());
                surface_areas_b.push_back(bound_curr_1.SurfaceArea());
            }
            unsigned long idx = 0;

            for (int i = 0; i < surface_areas_a.size()-1; ++i){
                sa = surface_areas_a[i];
                sb = surface_areas_b[surface_areas_b.size() - 1 - i];
                // std::cout << sa << " " << sb  << std::endl;

                double cost_tmp = sa * (i+1) + sb * (surface_areas_a.size() - 1 - i);
                if (cost_tmp < cost){
                    cost = cost_tmp;
                    idx = i;
                }
            }
            size1 = idx + 1;
            // std::cout << "\n";
        }
        else{
            size1 = (objects.size() / 2);
        }
            auto beginning = objects.begin();
            auto middling = objects.begin() + (size1);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        
        
        return node;
    }

    // return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    std::array<int, 3> dirIsPos = {(int) (ray.direction.x > 0), (int) (ray.direction.y > 0), (int) (ray.direction.z > 0)};
    Intersection intersect1, intersect2;
    if (! node->bounds.IntersectP(ray, ray.direction_inv, dirIsPos))
        return intersect1;
    
    if (node -> left){
        // has subnodes
        intersect1 = getIntersection(node->left, ray);
        intersect2 = getIntersection(node->right, ray);
        return intersect1.distance > intersect2.distance ? intersect2:intersect1;
    }
    else{ 
        //has no subnodes
        intersect1 = node->object->getIntersection(ray);
        return intersect1;
    }
    
    
}