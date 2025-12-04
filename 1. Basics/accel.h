#pragma once
#include "precomp.h"

using namespace Tmpl8;


struct Tri { float3 vertex0, vertex1, vertex2; float3 centroid; };
struct Node
{
	float3 aabbMin, aabbMax;
	uint leftFirst, triCount;
	bool isLeaf() { return triCount > 0; }
};
#define N	64

class Accel
{
public:
	virtual void Build();
	virtual void Trace(Ray& ray, const uint nodeIdx);
	bool IntersectAABB(const Ray& ray, const float3 bmin, const float3 bmax)
	{
		float tx1 = (bmin.x - ray.O.x) / ray.D.x, tx2 = (bmax.x - ray.O.x) / ray.D.x;
		float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
		float ty1 = (bmin.y - ray.O.y) / ray.D.y, ty2 = (bmax.y - ray.O.y) / ray.D.y;
		tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
		float tz1 = (bmin.z - ray.O.z) / ray.D.z, tz2 = (bmax.z - ray.O.z) / ray.D.z;
		tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
		return tmax >= tmin && tmin < ray.t && tmax > 0;
	}
	void IntersectTri(Ray& ray, const Tri& tri)
	{
		const float3 edge1 = tri.vertex1 - tri.vertex0;
		const float3 edge2 = tri.vertex2 - tri.vertex0;
		const float3 h = cross(ray.D, edge2);
		const float a = dot(edge1, h);
		if (a > -0.0001f && a < 0.0001f) return; // ray parallel to triangle
		const float f = 1 / a;
		const float3 s = ray.O - tri.vertex0;
		const float u = f * dot(s, h);
		if (u < 0 || u > 1) return;
		const float3 q = cross(s, edge1);
		const float v = f * dot(ray.D, q);
		if (v < 0 || u + v > 1) return;
		const float t = f * dot(edge2, q);
		if (t > 0.0001f) ray.t = min(ray.t, t);
	}
	virtual void UpdateNodeBounds(uint nodeIdx);
	virtual void Subdivide(uint nodeIdx);

public:
	Tri tri[N];
	uint triIdx[N];
	Node nodes[N * 2];
	uint rootNodeIdx = 0, nodesUsed = 1;
};