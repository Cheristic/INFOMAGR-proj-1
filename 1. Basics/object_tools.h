#pragma once

#include "precomp.h"

namespace Tmpl8 {
	struct Tri { float3 vertex0, vertex1, vertex2; float3 centroid; };
	// additional triangle data, for texturing and shading
	struct TriEx { float2 uv0, uv1, uv2; float3 N0, N1, N2; };
	struct Node
	{
		float3 aabbMin, aabbMax;
		uint leftFirst, triCount;
		bool isLeaf() { return triCount > 0; }
	};
	struct Intersection
	{
		float t = 1e34f;		// intersection distance along ray
		float u, v;		// barycentric coordinates of the intersection
		uint triIndex;	// triangle index
	};
	__declspec(align(64)) class Ray
	{
	public:
		Ray() = default;
		Ray(const float3 origin, const float3 direction, const float distance = 1e34f, const int idx = -1)
		{
			O = origin, D = direction, hit.t = distance;
			// calculate reciprocal ray direction for triangles and AABBs
			rD = float3(1 / D.x, 1 / D.y, 1 / D.z);
#ifdef SPEEDTRIX
			d0 = 1, d1 = d2 = 0; // ready for SIMD matrix math
#endif
		}
		float3 IntersectionPoint() const { return O + hit.t * D; }
		// ray data
#ifndef SPEEDTRIX
		float3 O, D, rD;
#else
		union { struct { float3 O; float d0; }; __m128 O4; };
		union { struct { float3 D; float d1; }; __m128 D4; };
		union { struct { float3 rD; float d2; }; __m128 rD4; };
#endif
		Intersection hit;
	};

	inline float3 RGB8toRGB32F(uint c)
	{
		float s = 1 / 256.0f;
		int r = (c >> 16) & 255;
		int g = (c >> 8) & 255;
		int b = c & 255;
		return float3(r * s, g * s, b * s);
	}

}