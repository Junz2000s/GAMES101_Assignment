//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>

class Bounds3
{
  public:
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    Vector3f Diagonal() const { return pMax - pMin; }
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};



inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    // invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
    // dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
    // TODO test if ray bound intersects
    Vector3f t_min = (pMin - ray.origin) * invDir;
    Vector3f t_max = (pMax - ray.origin) * invDir;
    float min_x, max_x, min_y, max_y, min_z, max_z;
    if (dirIsNeg[0]){
        min_x = t_min.x;
        max_x = t_max.x;
    }
    else{
        min_x = t_max.x;
        max_x = t_min.x;
    }
    if (dirIsNeg[1]){
        min_y = t_min.y;
        max_y = t_max.y;
    }
    else{
        min_y = t_max.y;
        max_y = t_min.y;
    }
    if (dirIsNeg[2]){
        min_z = t_min.z;
        max_z = t_max.z;
    }
    else{
        min_z = t_max.z;
        max_z = t_min.z;
    }
    float t_enter = std::max(min_x, std::max(min_y, min_z));
    float t_exit = std::min(max_x, std::min(max_y, max_z));
    return (t_exit >= t_enter) && (t_exit > 0);
    // float t_Min_x = (pMin.x - ray.origin.x)*invDir[0];
    // float t_Min_y = (pMin.y - ray.origin.y)*invDir[1];
    // float t_Min_z = (pMin.z - ray.origin.z)*invDir[2];
    // float t_Max_x = (pMax.x - ray.origin.x)*invDir[0];
    // float t_Max_y = (pMax.y - ray.origin.y)*invDir[1];
    // float t_Max_z = (pMax.z - ray.origin.z)*invDir[2];
    
    // //dirIsNeg表面光线的方向，如果是正方向则为1，pmin-O为最短路径
    // //反之为负方向0，pmax-O是最短路径
    // if(!dirIsNeg[0])
    // {
    //     float t = t_Min_x;
    //     t_Min_x = t_Max_x;
    //     t_Max_x = t;
    // }
    // if(!dirIsNeg[1])
    // {
    //     float t = t_Min_y;
    //     t_Min_y = t_Max_y;
    //     t_Max_y = t;
    // }
    // if(!dirIsNeg[2])
    // {
    //     float t = t_Min_z;
    //     t_Min_z = t_Max_z;
    //     t_Max_z = t;
    // }
 
    // float t_enter = std::max(t_Min_x,std::max(t_Min_y,t_Min_z));
    // float t_exit =  std::min(t_Max_x,std::min(t_Max_y,t_Max_z));
    // if(t_enter<t_exit&&t_exit>=0)
    //     return true;
    // else
    //     return false;
}

inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
