#pragma once

#include "object_tools.h"

namespace Tmpl8 {

	class Object
	{
	public:
		Object() = default;

		virtual bool IsOccluded(const Ray& ray) const { return false; }

		virtual float3 GetNormal(const float3 I) const { return float3(0.0f); }

		virtual float3 GetAlbedo(const float3 I) const { return float3(0.93f); }
	};

	class Plane : public Object
	{
	public:
		Plane() = default;
		Plane(int idx, float3 normal, float dist) : Nor(normal), d(dist) {}
		float3 GetNormal(const float3 I) const override
		{
			return Nor;
		}
		float3 GetAlbedo(const float3 I) const override
		{
			if (Nor.y == 1)
			{
				// floor albedo: checkerboard
				int ix = (int)(I.x * 2 + 96.01f);
				int iz = (int)(I.z * 2 + 96.01f);
				// add deliberate aliasing to two tile
				if (ix == 98 && iz == 98) ix = (int)(I.x * 32.01f), iz = (int)(I.z * 32.01f);
				if (ix == 94 && iz == 98) ix = (int)(I.x * 64.01f), iz = (int)(I.z * 64.01f);
				return float3(((ix + iz) & 1) ? 1 : 0.3f);
			}
			else if (Nor.z == -1)
			{
				// back wall: logo
				static Surface logo("../assets/logo.png");
				int ix = (int)((I.x + 4) * (128.0f / 8)), iy = (int)((2 - I.y) * (64.0f / 3));
				uint p = logo.pixels[(ix & 127) + (iy & 63) * 128];
				uint3 i3((p >> 16) & 255, (p >> 8) & 255, p & 255);
				return float3(i3) * (1.0f / 255.0f);
			}
			else if (Nor.x == 1)
			{
				// left wall: red
				static Surface red("../assets/red.png");
				int ix = (int)((I.z - 4) * (512.0f / 7)), iy = (int)((2 - I.y) * (512.0f / 3));
				uint p = red.pixels[(ix & 511) + (iy & 511) * 512];
				uint3 i3((p >> 16) & 255, (p >> 8) & 255, p & 255);
				return float3(i3) * (1.0f / 255.0f);
			}
			else if (Nor.x == -1)
			{
				// right wall: blue
				static Surface blue("../assets/blue.png");
				int ix = (int)((I.z - 4) * (512.0f / 7)), iy = (int)((2 - I.y) * (512.0f / 3));
				uint p = blue.pixels[(ix & 511) + (iy & 511) * 512];
				uint3 i3((p >> 16) & 255, (p >> 8) & 255, p & 255);
				return float3(i3) * (1.0f / 255.0f);
			}
			return float3(0.93f);
		}
		float3 Nor;
		float d;
	};

	class Quad : public Object
	{
	public:
		Quad() = default;
		Quad(float s, mat4 transform = mat4::Identity())
		{
			size = s * 0.5f;
			T = transform, invT = transform.FastInvertedTransformNoScale();
		}
		float3 GetNormal(const float3 I) const override
		{
			// TransformVector( float3( 0, -1, 0 ), T ) 
			return float3(-T.cell[1], -T.cell[5], -T.cell[9]);
		}
		float3 GetAlbedo(const float3 I) const override
		{
			return float3(10);
		}
		float size;
		mat4 T, invT;
	};


}