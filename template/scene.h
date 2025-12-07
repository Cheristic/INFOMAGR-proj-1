#pragma once
#include "objects.h"
#include "bvh.h"
#include "kdtree.h"

// -----------------------------------------------------------
// scene.h
// Simple test scene for ray tracing experiments. Goals:
// - Super-fast scene intersection
// - Easy interface: scene.FindNearest / IsOccluded
// - With normals and albedo: GetNormal / GetAlbedo
// - Area light source (animated), for light transport
// - Primitives can be hit from inside - for dielectrics
// - Can be extended with other primitives and/or a BVH
// - Optionally animated - for temporal experiments
// - Not everything is axis aligned - for cache experiments
// - Can be evaluated at arbitrary time - for motion blur
// - Has some high-frequency details - for filtering
// Some speed tricks that severely affect maintainability
// are enclosed in #ifdef SPEEDTRIX / #endif. Mind these
// if you plan to alter the scene in any way.
// -----------------------------------------------------------

namespace Tmpl8 {

// -----------------------------------------------------------
// Scene class
// We intersect this. The query is internally forwarded to the
// list of primitives, so that the nearest hit can be returned.
// For this hit (distance, obj id), we can query the normal and
// albedo.
// -----------------------------------------------------------
	class Scene
	{
	public:
		Scene()
		{
			uint objIdx = 0;
			// we store all primitives in one continuous buffer
			for (int i = 0; i < 4; i++) lights[i] = Quad(objIdx, 5.0f);	// 0: four light sources

			if (SceneIdx == 0) 
			{
				plane[0] = Plane(++objIdx, float3(1, 0, 0), 3);			// 4: left wall
				plane[1] = Plane(++objIdx, float3(-1, 0, 0), 2.99f);		// 5: right wall
				plane[2] = Plane(++objIdx, float3(0, 1, 0), 1);			// 6: floor
				plane[3] = Plane(++objIdx, float3(0, -1, 0), 2);			// 7: ceiling
				plane[4] = Plane(++objIdx, float3(0, 0, 1), 3);			// 8: front wall
				plane[5] = Plane(++objIdx, float3(0, 0, -1), 3.99f);		// 9: back wall

				int currIdx = objIdx;
				bvh = BVH("../assets/teapot.obj", &objIdx, 1);
				bvh.Build();
				objIdx = currIdx;
				kdtree = KDTree("../assets/teapot.obj",&objIdx, 1);
				kdtree.Build();

				bvh.M = mat4::Translate(-0.25f, 0, 2) * mat4::RotateX(PI / 4);
				bvh.invM = bvh.M.Inverted();
			}
			else if (SceneIdx == 1)
			{
				int currIdx = objIdx;
				bvh = BVH("../assets/teapot.obj", &objIdx, 1);
				bvh.Build();
				objIdx = currIdx;
				kdtree = KDTree("../assets/teapot.obj", &objIdx, 1);
				kdtree.Build();

				firstAccel2_objIdx = objIdx;

				bvh = BVH("../assets/teapot.obj", &objIdx, 1);
				bvh.Build();
				objIdx = firstAccel2_objIdx;
				kdtree = KDTree("../assets/teapot.obj", &objIdx, 1);
				kdtree.Build();
			}


			SetTime(0);
			// Note: once we have triangle support we should get rid of the class
			// hierarchy: virtuals reduce performance somewhat.
		}
		void SetTime(float t)
		{
			// default time for the scene is simply 0. Updating/ the time PER frame 
			// enables animation. Updating it per ray can be used for motion blur.
			animTime = t;

			// four light sources are stationary
			lights[0].T = mat4::Translate(-1, 1.5f, -1), lights[0].invT = lights[0].T.FastInvertedTransformNoScale();
			lights[1].T = mat4::Translate(1, 1.5f, -1), lights[1].invT = lights[1].T.FastInvertedTransformNoScale();
			lights[2].T = mat4::Translate(1, 1.5f, 1), lights[2].invT = lights[2].T.FastInvertedTransformNoScale();
			lights[3].T = mat4::Translate(-1, 1.5f, 1), lights[3].invT = lights[3].T.FastInvertedTransformNoScale();

		}
		float3 GetLightPos() const
		{
			// function is not valid when using four lights; we'll return the origin
			return float3(0);
		}
		float3 RandomPointOnLight(const float r0, const float r1)
		{
			// select a random light and use that
			uint lightIdx = (uint)(r0 * 4);
			const Quad& q = lights[lightIdx];
			// renormalize r0 for reuse
			float stratum = lightIdx * 0.25f;
			float r2 = (r0 - stratum) / (1 - stratum);
			// get a random position on the selected quad
			const float size = q.size;
			float3 corner1 = TransformPosition(float3(-size, 0, -size), q.T);
			float3 corner2 = TransformPosition(float3(size, 0, -size), q.T);
			float3 corner3 = TransformPosition(float3(-size, 0, size), q.T);
			return corner1 + r2 * (corner2 - corner1) + r1 * (corner3 - corner1);
		}
		uint GetRandomLight(uint& seed) 
		{
			float r = RandomFloat();
			uint lightIdx = (uint)(r * 4);
			return lightIdx;
		}
		float3 RandomPointOnLightQuad(uint lightIdx, uint& seed) {
			const float r0 = RandomFloat();
			const float r1 = RandomFloat();
			const Quad& q = lights[lightIdx];
			float stratum = lightIdx * 0.25f;
			float r2 = (r0 - stratum) / (1 - stratum);
			// get a random position on the selected quad
			const float size = q.size;
			float3 corner1 = TransformPosition(float3(-size, 0, -size), q.T);
			float3 corner2 = TransformPosition(float3(size, 0, -size), q.T);
			float3 corner3 = TransformPosition(float3(-size, 0, size), q.T);
			return corner1 + r2 * (corner2 - corner1) + r1 * (corner3 - corner1);
		}
		Quad GetLightQuad(uint lightIdx) {
			return lights[lightIdx];
		}
		float3 RandomPointOnLight(uint& seed)
		{
			return RandomPointOnLight(RandomFloat(seed), RandomFloat(seed));
		}
		void GetLightQuad(float3& v0, float3& v1, float3& v2, float3& v3, const uint idx = 0)
		{

			// return four corners of the specified light
			const Quad& q = lights[idx];
			const float size = q.size;
			v0 = TransformPosition(float3(-size, 0, size), q.T);
			v1 = TransformPosition(float3(size, 0, size), q.T);
			v2 = TransformPosition(float3(size, 0, -size), q.T);
			v3 = TransformPosition(float3(-size, 0, -size), q.T);
		}
		float3 GetLightColor() const
		{
			return float3(24, 24, 22);
		}
		float3 GetAreaLightColor() const
		{
			return lights[0].GetAlbedo(float3(0)); // they're all the same color
		}
		float GetLightArea() const
		{
			return sqrf(lights[0].size * 2); // all the same size
		}
		constexpr float GetLightCount() const
		{
			return 4; // what did you expect
		}
		void FindNearest(Ray& ray)
		{
			// room walls - ugly shortcut for more speed
			// 
			// TODO: the room is actually just an AABB; use slab test
			if (SceneIdx == 0) 
			{
				static const __m128 x4min = _mm_setr_ps(3, 1, 3, 1e30f);
				static const __m128 x4max = _mm_setr_ps(-2.99f, -2, -3.99f, 1e30f);
				static const __m128 idmin = _mm_castsi128_ps(_mm_setr_epi32(4, 6, 8, -1));
				static const __m128 idmax = _mm_castsi128_ps(_mm_setr_epi32(5, 7, 9, -1));
				static const __m128 zero4 = _mm_setzero_ps();
				const __m128 selmask = _mm_cmpge_ps(ray.D4, zero4);
				const __m128i idx4 = _mm_castps_si128(_mm_blendv_ps(idmin, idmax, selmask));
				const __m128 x4 = _mm_blendv_ps(x4min, x4max, selmask);
				const __m128 d4 = _mm_sub_ps(zero4, _mm_mul_ps(_mm_add_ps(ray.O4, x4), ray.rD4));
				const __m128 mask4 = _mm_cmple_ps(d4, zero4);
				const __m128 t4 = _mm_blendv_ps(d4, _mm_set1_ps(1e34f), mask4);
				/* first: unconditional */  ray.t = t4.m128_f32[0], ray.objIdx = idx4.m128i_i32[0];
				if (t4.m128_f32[1] < ray.t) ray.t = t4.m128_f32[1], ray.objIdx = idx4.m128i_i32[1];
				if (t4.m128_f32[2] < ray.t) ray.t = t4.m128_f32[2], ray.objIdx = idx4.m128i_i32[2];
			}


			// efficient four-quad intersection by Jesse Vrooman
			const __m128 t = _mm_div_ps(_mm_add_ps(_mm_set1_ps(ray.O.y),
				_mm_set1_ps(-1.5)), _mm_xor_ps(_mm_set1_ps(ray.D.y), _mm_set1_ps(-0.0)));
			const __m128 Ix = _mm_add_ps(_mm_add_ps(_mm_set1_ps(ray.O.x),
				_mm_set_ps(1, -1, -1, 1)), _mm_mul_ps(t, _mm_set1_ps(ray.D.x)));
			const __m128 Iz = _mm_add_ps(_mm_add_ps(_mm_set1_ps(ray.O.z),
				_mm_set_ps(1, 1, -1, -1)), _mm_mul_ps(t, _mm_set1_ps(ray.D.z)));
			const static __m128 size = _mm_set1_ps(0.25f);
			const static __m128 nsize = _mm_xor_ps(_mm_set1_ps(0.25f), _mm_set1_ps(-0.0));
			const __m128 maskedT = _mm_and_ps(t, _mm_and_ps(
				_mm_and_ps(_mm_cmpgt_ps(Ix, nsize), _mm_cmplt_ps(Ix, size)),
				_mm_and_ps(_mm_cmpgt_ps(Iz, nsize), _mm_cmplt_ps(Iz, size))));
			if (maskedT.m128_f32[3] > 0) ray.t = maskedT.m128_f32[3], ray.objIdx = 0;
			if (maskedT.m128_f32[2] > 0) ray.t = maskedT.m128_f32[2], ray.objIdx = 0;
			if (maskedT.m128_f32[1] > 0) ray.t = maskedT.m128_f32[1], ray.objIdx = 0;
			if (maskedT.m128_f32[0] > 0) ray.t = maskedT.m128_f32[0], ray.objIdx = 0;

			if (SceneIdx == 0) {
				if (useBVH) bvh.Intersect(ray, bvh.rootNodeIdx);
				else kdtree.Intersect(ray, kdtree.rootNodeIdx);
			}
			else if (SceneIdx == 1) {
				if (useBVH)
				{
					bvh.Intersect(ray, bvh.rootNodeIdx); bvh2.Intersect(ray, bvh2.rootNodeIdx);
				}
				else
				{
					kdtree.Intersect(ray, kdtree.rootNodeIdx); kdtree2.Intersect(ray, kdtree2.rootNodeIdx);
				}
			}


		}
		bool IsOccluded(const Ray& ray) const
		{
			for (int i = 0; i < 4; i++) if (lights[i].IsOccluded(ray)) return true;
			return false; // skip planes and rounded corners
		}
		float3 GetNormal(const int objIdx, const float3 I, const float3 wo) const
		{
			// we get the normal after finding the nearest intersection:
			// this way we prevent calculating it multiple times.
			if (objIdx == -1) return float3(0); // or perhaps we should just crash
			float3 N;
			if (objIdx == 0) N = lights[0].GetNormal(I); // they're all oriented the same

			if (SceneIdx == 0) {
				if (objIdx >= 4 && objIdx <= 9) {
					// faster to handle the 6 planes without a call to GetNormal
					N = float3(0);
					N[(objIdx - 4) / 2] = 1 - 2 * (float)(objIdx & 1);
				}
				else
				{
					if (useBVH) N = bvh.GetNormal(objIdx);
					else N = kdtree.GetNormal(objIdx);
				}
			}
			else if (SceneIdx == 1) {
				if (objIdx < firstAccel2_objIdx) {
					if (useBVH) N = bvh.GetNormal(objIdx);
					else N = kdtree.GetNormal(objIdx);
				}
				else {
					if (useBVH) N = bvh2.GetNormal(objIdx);
					else N = kdtree2.GetNormal(objIdx);
				}
			}
			
			if (dot(N, wo) > 0) N = -N; // hit backside / inside
			return N;
		}
		float3 GetAlbedo(int objIdx, float3 I) const
		{
			if (objIdx == -1) return float3(0); // or perhaps we should just crash
			if (objIdx == 0) return lights[0].GetAlbedo(I); // they're all the same

			if (SceneIdx == 0) {
				if (objIdx >= 4 && objIdx <= 9) return plane[objIdx - 4].GetAlbedo(I);
				else if (useBVH) return bvh.GetAlbedo();
				else return kdtree.GetAlbedo();
			}
			else if (SceneIdx == 1) {
				if (objIdx < firstAccel2_objIdx) {
					if (useBVH) return bvh.GetAlbedo();
					else return kdtree.GetAlbedo();
				}
				else {
					if (useBVH) return bvh2.GetAlbedo();
					else return kdtree2.GetAlbedo();
				}
			}
			return 0;
		}

		float3 GetCameraPos(int posIdx) {
			if (SceneIdx == 0) 
			{
				if (posIdx == 0) return float3(0, 0, -2);
			}
			else if (SceneIdx == 1) {
				if (posIdx == 0) return float3(0, 0, -2);
			}
			return 0;
		}

		float3 GetCameraTarget(int posIdx) {
			if (SceneIdx == 0)
			{
				if (posIdx == 0) return float3(0, 0, -1);
			}
			else if (SceneIdx == 1) {
				if (posIdx == 0) return float3(0, 0, -1);
			}
			return 0;
		}

		__declspec(align(64)) // start a new cacheline here
			float animTime = 0;

		Quad lights[4];
		Plane plane[6];

		bool useBVH = true;
		BVH bvh;
		KDTree kdtree;

		int firstAccel2_objIdx = -1;
		BVH bvh2;
		KDTree kdtree2;

		int SceneIdx = 0;
	};
}