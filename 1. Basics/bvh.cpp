#include "accel.h"

class BVH : public Accel 
{
public:
	void Build()
	{
		// populate triangle index array
		for (int i = 0; i < N; i++) triIdx[i] = i;
		// calculate triangle centroids for partitioning
		for (int i = 0; i < N; i++)
			tri[i].centroid = (tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2) * 0.3333f;
		// assign all triangles to root node
		Node& root = nodes[rootNodeIdx];
		root.leftFirst = 0, root.triCount = N;
		UpdateNodeBounds(rootNodeIdx);
		// subdivide recursively
		Subdivide(rootNodeIdx);
	}
	void Trace(Ray& ray, const uint nodeIdx)
	{
		Node& node = nodes[nodeIdx];
		if (!IntersectAABB(ray, node.aabbMin, node.aabbMax)) return;
		if (node.isLeaf())
		{
			for (uint i = 0; i < node.triCount; i++)
				IntersectTri(ray, tri[triIdx[node.leftFirst + i]]);
		}
		else
		{
			Trace(ray, node.leftFirst);
			Trace(ray, node.leftFirst + 1);
		}
	}
	void UpdateNodeBounds(uint nodeIdx)
	{
		Node& node = nodes[nodeIdx];
		node.aabbMin = float3(1e30f);
		node.aabbMax = float3(-1e30f);
		for (uint first = node.leftFirst, i = 0; i < node.triCount; i++)
		{
			uint leafTriIdx = triIdx[first + i];
			Tri& leafTri = tri[leafTriIdx];
			node.aabbMin = fminf(node.aabbMin, leafTri.vertex0),
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex1),
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex2),
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex0),
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex1),
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex2);
		}
	}
	void Subdivide(uint nodeIdx)
	{
		// terminate recursion
		Node& node = nodes[nodeIdx];
		if (node.triCount <= 2) return;
		// determine split axis and position
		float3 extent = node.aabbMax - node.aabbMin;
		int axis = 0;
		if (extent.y > extent.x) axis = 1;
		if (extent.z > extent[axis]) axis = 2;
		float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f;
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
		UpdateNodeBounds(leftChildIdx);
		UpdateNodeBounds(rightChildIdx);
		// recurse
		Subdivide(leftChildIdx);
		Subdivide(rightChildIdx);
	}
};