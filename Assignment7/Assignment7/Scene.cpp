//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    auto inter = intersect(ray);
    if (! (inter.happened)) 
        return Vector3f(0);
    
    if (!(inter.obj->hasEmit())){
        return shade(inter, normalize(ray.direction));
    }
    else{ // the first intersection is on the light
        // std::cout << inter.m->getEmission() / ((inter.distance) * (inter.distance)) << std::endl;
        return inter.m->getEmission(); 
    }
}

Vector3f Scene::shade(const Intersection& inter, Vector3f dir) const
{
    Vector3f L_dir(0), L_indir(0);
    
    // Sample the direct light
    Intersection pos_light;
    float pdf;
    sampleLight(pos_light, pdf);
    auto w_in_dir = normalize(inter.coords - pos_light.coords); // light to object
    Ray ray_dir(pos_light.coords, w_in_dir);
    auto inter_check = intersect(ray_dir);
    if (inter_check.happened && (inter_check.coords - inter.coords).norm() < EPSILON){
        // make sure that the direct light is not blocked

        auto fr = inter.m->eval(w_in_dir, -dir, inter.normal);
        auto Li = pos_light.emit; // defined in Meshtriangle and Sphere class, here the light is MeshTriangle class
        auto dist = (pos_light.coords - inter.coords).norm();
        
        auto cos_theta_A = dotProduct(w_in_dir, normalize(pos_light.normal));
        // about why there can be negative: the light is slightly lower than the ceiling
        
        auto cos_theta_i = dotProduct(- w_in_dir, normalize(inter.normal));
        // std::cout << cos_theta_A << " " << cos_theta_i << std::endl;
        L_dir += Li * fr * cos_theta_A * cos_theta_i / (dist * dist) / pdf;
        // std::cout << Li << " " << fr << " " << dist << std::endl;
    }

    // indirect light part
    if (get_random_float() < RussianRoulette){
        auto w_in_indirect = inter.m->sample(dir, normalize(inter.normal)).normalized();
        // we found that w_in_indirect is pointed out of the surface, and it has been normalized
        
        pdf = inter.m->pdf(w_in_indirect, - dir, normalize(inter.normal));
        
        Intersection inter_nxt_obj = intersect(Ray(inter.coords, w_in_indirect));
        if (inter_nxt_obj.happened && (!inter_nxt_obj.obj->hasEmit()))
        {
            auto next_light_intense = shade(inter_nxt_obj, w_in_indirect);
            auto cos_theta_i = dotProduct(w_in_indirect, normalize(inter.normal));
            auto fr = inter.m->eval(w_in_indirect, -dir, normalize(inter.normal));
            L_indir = next_light_intense * cos_theta_i * fr / pdf / RussianRoulette;

        }
    }
    return L_dir + L_indir;

}