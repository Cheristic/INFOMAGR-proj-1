#pragma once
#include "precomp.h"
#include "scene.h"


namespace Tmpl8 {

	class Accel
	{
	public:

		void Build();
		float3 Trace(Ray& ray, const uint nodeIdx);
		void Subdivide(uint nodeIdx);

		bool IntersectAABB(Ray& ray, const float3 bmin, const float3 bmax)
		{
			float tx1 = (bmin.x - ray.O.x) / ray.D.x, tx2 = (bmax.x - ray.O.x) / ray.D.x;
			float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
			float ty1 = (bmin.y - ray.O.y) / ray.D.y, ty2 = (bmax.y - ray.O.y) / ray.D.y;
			tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
			float tz1 = (bmin.z - ray.O.z) / ray.D.z, tz2 = (bmax.z - ray.O.z) / ray.D.z;
			tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
			return tmax >= tmin && tmin < ray.hit.t && tmax > 0;
		}
		void IntersectTri(Ray& ray, const Tri& tri, const uint triIndex)
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
			if (t > 0.0001f && t < ray.hit.t)
				ray.hit.t = t, ray.hit.u = u,
				ray.hit.v = v, ray.hit.triIndex = triIndex;
		}
		float EvaluateSAH(Node& node, int axis, float pos)
		{
			// determine triangle counts and bounds for this split candidate
			aabb leftBox, rightBox;
			int leftCount = 0, rightCount = 0;
			for (uint i = 0; i < node.triCount; i++)
			{
				Tri& triangle = tri[triIdx[node.leftFirst + i]];
				if (triangle.centroid[axis] < pos)
				{
					leftCount++;
					leftBox.Grow(triangle.vertex0);
					leftBox.Grow(triangle.vertex1);
					leftBox.Grow(triangle.vertex2);
				}
				else
				{
					rightCount++;
					rightBox.Grow(triangle.vertex0);
					rightBox.Grow(triangle.vertex1);
					rightBox.Grow(triangle.vertex2);
				}
			}
			float cost = leftCount * leftBox.Area() + rightCount * rightBox.Area();
			return cost > 0 ? cost : 1e30f;
		}

		Tri tri[MAX_TRIS];
		uint triIdx[MAX_TRIS];
		TriEx triEx[MAX_TRIS];
		Node nodes[MAX_TRIS * 2];
		uint rootNodeIdx = 0, nodesUsed = 1;
		uint triCount;
	};
}