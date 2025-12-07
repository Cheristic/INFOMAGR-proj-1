#pragma once

#include "precomp.h"
#define SPEEDTRIX

#define PLANE_X(o,i) {t=-(ray.O.x+o)*ray.rD.x;if(t<ray.t&&t>0)ray.t=t,ray.objIdx=i;}
#define PLANE_Y(o,i) {t=-(ray.O.y+o)*ray.rD.y;if(t<ray.t&&t>0)ray.t=t,ray.objIdx=i;}
#define PLANE_Z(o,i) {t=-(ray.O.z+o)*ray.rD.z;if(t<ray.t&&t>0)ray.t=t,ray.objIdx=i;}

namespace Tmpl8 {

	struct Node
	{
		float3 aabbMin, aabbMax;
		uint leftFirst, triCount;
		bool isLeaf() { return triCount > 0; }
	};
	
	__declspec(align(64)) class Ray
	{
	public:
		Ray() = default;
		Ray(const float3 origin, const float3 direction, const float distance = 1e30f, const int idx = -1)
		{
			O = origin, D = direction, t = distance;
			// calculate reciprocal ray direction for triangles and AABBs
			rD = float3(1 / D.x, 1 / D.y, 1 / D.z);
#ifdef SPEEDTRIX
			d0 = 1, d1 = d2 = 0; // ready for SIMD matrix math
#endif
			objIdx = idx;
		}
		float3 IntersectionPoint() const { return O + t * D; }
		// ray data
#ifndef SPEEDTRIX
		float3 O, D, rD;
#else
		union { struct { float3 O; float d0; }; __m128 O4; };
		union { struct { float3 D; float d1; }; __m128 D4; };
		union { struct { float3 rD; float d2; }; __m128 rD4; };
#endif
		float t = 1e30f;
		int objIdx = -1;
		bool inside = false; // true when in medium
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