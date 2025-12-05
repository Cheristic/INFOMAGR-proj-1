#include "accel.h"

class KDTree : public Accel
{
public:
	void KDTree::Build()
	{
		// populate triangle index array
		for (int i = 0; i < MAX_TRIS; i++) triIdx[i] = i;
		// calculate triangle centroids for partitioning
		for (int i = 0; i < MAX_TRIS; i++)
			tri[i].centroid = (tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2) * 0.3333f;
		// assign all triangles to root node
		Node& root = nodes[rootNodeIdx];
		root.leftFirst = 0, root.triCount = MAX_TRIS;
		// get bounds
		root.aabbMin = float3(1e30f);
		root.aabbMax = float3(-1e30f);
		for (uint first = root.leftFirst, i = 0; i < root.triCount; i++)
		{
			uint leafTriIdx = triIdx[first + i];
			Tri& leafTri = tri[leafTriIdx];
			root.aabbMin = fminf(root.aabbMin, leafTri.vertex0),
			root.aabbMin = fminf(root.aabbMin, leafTri.vertex1),
			root.aabbMin = fminf(root.aabbMin, leafTri.vertex2),
			root.aabbMax = fmaxf(root.aabbMax, leafTri.vertex0),
			root.aabbMax = fmaxf(root.aabbMax, leafTri.vertex1),
			root.aabbMax = fmaxf(root.aabbMax, leafTri.vertex2);
		}
		// subdivide recursively
		Subdivide(rootNodeIdx);
	}
	float3 KDTree::Trace(Ray& ray, const uint nodeIdx)
	{
		Intersect(ray, nodeIdx);
		Intersection i = ray.hit;
		if (i.t == 1e34f)
		{
			return float3(0);
		}
		// calculate texture uv based on barycentrics
		uint triIdx = i.triIndex;
		TriEx& tri = triEx[triIdx];
		Surface* tex = &Surface("../assets/logo.png");
		float2 uv = i.u * tri.uv1 + i.v * tri.uv2 + (1 - (i.u + i.v)) * tri.uv0;
		int iu = (int)(uv.x * tex->width) % tex->width;
		int iv = (int)(uv.y * tex->height) % tex->height;
		uint texel = tex->pixels[iu + iv * tex->width];
		float3 albedo = RGB8toRGB32F(texel);
		// calculate the normal for the intersection
		float3 N = i.u * tri.N1 + i.v * tri.N2 + (1 - (i.u + i.v)) * tri.N0;
		float3 I = ray.O + i.t * ray.D;
		// shading
		// calculate the diffuse reflection in the intersection point
		float3 lightPos(3, 10, 2);
		float3 lightColor(150, 150, 120);
		float3 ambient(0.2f, 0.2f, 0.4f);
		float3 L = lightPos - I;
		float dist = length(L);
		L *= 1.0f / dist;
		return albedo * (ambient + max(0.0f, dot(N, L)) * lightColor * (1.0f / (dist * dist)));
	}
	void Intersect(Ray& ray, const uint nodeIdx)
	{
		Node& node = nodes[nodeIdx];
		if (!IntersectAABB(ray, node.aabbMin, node.aabbMax)) return;
		if (node.isLeaf())
		{
			for (uint i = 0; i < node.triCount; i++)
				IntersectTri(ray, tri[triIdx[node.leftFirst + i]], triIdx[node.leftFirst + i]);
		}
		else
		{
			Intersect(ray, node.leftFirst);
			Intersect(ray, node.leftFirst + 1);
		}
	}

	void KDTree::Subdivide(uint nodeIdx)
	{
		// terminate recursion
		Node& node = nodes[nodeIdx];
		// determine split axis using SAH
		int bestAxis = -1;
		float bestPos = 0, bestCost = 1e30f;
		for (int axis = 0; axis < 3; axis++) for (uint i = 0; i < node.triCount; i++)
		{
			Tri& triangle = tri[triIdx[node.leftFirst + i]];
			float candidatePos = triangle.centroid[axis];
			float cost = EvaluateSAH(node, axis, candidatePos);
			if (cost < bestCost)
				bestPos = candidatePos, bestAxis = axis, bestCost = cost;
		}
		int axis = bestAxis;
		float splitPos = bestPos;
		float3 e = node.aabbMax - node.aabbMin; // extent of parent
		float parentArea = e.x * e.y + e.y * e.z + e.z * e.x;
		float parentCost = node.triCount * parentArea;
		if (bestCost >= parentCost) return;
		// in-place partition
		int i = node.leftFirst;
		int j = i + node.triCount - 1;
		while (i <= j)
		{
			if (tri[triIdx[i]].centroid[axis] < splitPos)
				i++;
			else
				swap(triIdx[i], triIdx[j--]);
		}
		// abort split if one of the sides is empty
		int leftCount = i - node.leftFirst;
		if (leftCount == 0 || leftCount == node.triCount) return;
		// create child nodes
		int leftChildIdx = nodesUsed++;
		int rightChildIdx = nodesUsed++;
		nodes[leftChildIdx].leftFirst = node.leftFirst;
		nodes[leftChildIdx].triCount = leftCount;
		nodes[rightChildIdx].leftFirst = i;
		nodes[rightChildIdx].triCount = node.triCount - leftCount;
		node.leftFirst = leftChildIdx;
		node.triCount = 0;
		UpdateNodeBounds(leftChildIdx, nodeIdx, node.aabbMin[axis], splitPos, axis);
		UpdateNodeBounds(rightChildIdx, nodeIdx, splitPos, node.aabbMax[axis], axis);
		// recurse
		Subdivide(leftChildIdx);
		Subdivide(rightChildIdx);
	}
	void KDTree::UpdateNodeBounds(uint nodeIdx, uint parNodeIdx, float min, float max, int axis)
	{
		Node& parNode = nodes[parNodeIdx];
		Node& node = nodes[nodeIdx];
		node.aabbMin = parNode.aabbMin;
		node.aabbMax = parNode.aabbMax;
		node.aabbMin[axis] = min;
		node.aabbMax[axis] = max;
	}
};