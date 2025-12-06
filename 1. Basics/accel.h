#pragma once
#include "precomp.h"
#include <windows.h> 


namespace Tmpl8 {

	class Accel
	{
	public:
		Accel(const char * objFile, uint* objIdxTracker, const float scale = 1)
		{
			FILE* file = fopen(objFile, "r");
			float a, b, c, d, e, f, g, h, i;
			if (!file) return; // file doesn't exist

			while (fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
				&a, &b, &c, &d, &e, &f, &g, &h, &i) == 9)
			{
				tri[triCount].vertex0 = float3(a, b, c);
				tri[triCount].vertex1 = float3(d, e, f);
				tri[triCount++].vertex2 = float3(g, h, i);
				tri->objIdx = *objIdxTracker++;
			}
			fclose(file);

			nodes = new Node[triCount*2];
			triIdx = new uint[triCount];
		}
		void Build();
		void Subdivide(uint nodeIdx);
		void Intersect(Ray& ray, uint nodeIdx);

		float IntersectAABB(Ray& ray, const float3 bmin, const float3 bmax) const
		{
			float tx1 = (bmin.x - ray.O.x) / ray.D.x, tx2 = (bmax.x - ray.O.x) / ray.D.x;
			float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
			float ty1 = (bmin.y - ray.O.y) / ray.D.y, ty2 = (bmax.y - ray.O.y) / ray.D.y;
			tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
			float tz1 = (bmin.z - ray.O.z) / ray.D.z, tz2 = (bmax.z - ray.O.z) / ray.D.z;
			tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
			if (tmax >= tmin && tmin < ray.t && tmax > 0) return tmin;
			else return 1e30f;
		}

		void IntersectTri(Ray& ray, const Tri& tri) const
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
			if (t > 0.0001f && t < ray.t) ray.t = t;
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
		uint* triIdx = 0;
		Node* nodes = 0;
		uint rootNodeIdx = 0, nodesUsed = 1;
		uint triCount = 0;
	};
}