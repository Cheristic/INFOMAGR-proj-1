#pragma once
#include "precomp.h"
#include <windows.h> 


namespace Tmpl8 {

	class Accel
	{
	public:
		Accel() = default;
		Accel(const char * objFile, uint* objIdxTracker, const float scale = 1, mat4 transform = mat4::Identity())
		{
			FILE* file = fopen(objFile, "r");
			tri = new Tri[MAX_TRIS];
			N = new float3[MAX_TRIS], P = new float3[MAX_TRIS];
			M = transform, invM = transform.FastInvertedTransformNoScale();

			int Ns = 0, Ps = 0, a, b, c, d, e, f, g, h, i;
			if (!file) return; // file doesn't exist
			while (!feof(file))
			{
				char line[512] = { 0 };
				fgets(line, 511, file);
				if (line == strstr(line, "vn "))
					sscanf(line + 3, "%f %f %f", &N[Ns].x, &N[Ns].y, &N[Ns].z), Ns++;
				else if (line[0] == 'v')
					sscanf(line + 2, "%f %f %f", &P[Ps].x, &P[Ps].y, &P[Ps].z), Ps++;
				if (line[0] != 'f') continue; else
					sscanf(line + 2, "%i/%i/%i %i/%i/%i %i/%i/%i",
						&a, &b, &c, &d, &e, &f, &g, &h, &i);
				tri[triCount].vertexIdx0 = a - 1, tri[triCount].normalIdx0 = c - 1;
				tri[triCount].vertexIdx1 = d - 1, tri[triCount].normalIdx1 = f - 1;
				tri[triCount].vertexIdx2 = g - 1, tri[triCount].normalIdx2 = i - 1;
				tri[triCount++].objIdx = (*objIdxTracker);
				(*objIdxTracker)++;
			}

			fclose(file);

			nodes = new Node[triCount*2];
			triIdx = new uint[triCount];
		}
		void Build();
		void Subdivide(uint nodeIdx);
		void Intersect(Ray& ray, uint nodeIdx);

		float3 GetAlbedo() const
		{
			return float3(1);
		}

		float3 GetNormal(uint objIdx) const
		{
			for (int i = 0; i < MAX_TRIS; i++) {
				if (tri[i].objIdx == objIdx) {
					return (N[tri[i].normalIdx0] + N[tri[i].normalIdx1] + N[tri[i].normalIdx2]) * 0.3333f;
				}
			}
			return float3(1);
		}

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
			const float3 edge1 = P[tri.vertexIdx1] - P[tri.vertexIdx0];
			const float3 edge2 = P[tri.vertexIdx2] - P[tri.vertexIdx0];
			const float3 h = cross(ray.D, edge2);
			const float a = dot(edge1, h);
			if (a > -0.0001f && a < 0.0001f) return; // ray parallel to triangle
			const float f = 1 / a;
			const float3 s = ray.O - P[tri.vertexIdx0];
			const float u = f * dot(s, h);
			if (u < 0 || u > 1) return;
			const float3 q = cross(s, edge1);
			const float v = f * dot(ray.D, q);
			if (v < 0 || u + v > 1) return;
			const float t = f * dot(edge2, q);
			if (t > 0.0001f && t < ray.t) 
				ray.t = t; ray.objIdx = tri.objIdx;
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
					leftBox.Grow(P[triangle.vertexIdx0]);
					leftBox.Grow(P[triangle.vertexIdx1]);
					leftBox.Grow(P[triangle.vertexIdx2]);
				}
				else
				{
					rightCount++;
					rightBox.Grow(P[triangle.vertexIdx0]);
					rightBox.Grow(P[triangle.vertexIdx1]);
					rightBox.Grow(P[triangle.vertexIdx2]);
				}
			}
			float cost = leftCount * leftBox.Area() + rightCount * rightBox.Area();
			return cost > 0 ? cost : 1e30f;
		}

		Tri* tri;
		uint* triIdx = 0;
		Node* nodes = 0;
		int rootNodeIdx = 0, nodesUsed = 1;
		uint triCount = 0;
		float3* P = 0, * N = 0;
		mat4 M, invM;
	};
}