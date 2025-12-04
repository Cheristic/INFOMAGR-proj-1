#include "accel.h"

class KDTree : public Accel
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
	float3 Trace(Ray& ray, const uint nodeIdx)
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
	void Subdivide(uint nodeIdx)
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
	void UpdateNodeBounds(uint nodeIdx, uint parNodeIdx, float min, float max, int axis)
	{
		Node& parNode = nodes[parNodeIdx];
		Node& node = nodes[nodeIdx];
		node.aabbMin = parNode.aabbMin;
		node.aabbMax = parNode.aabbMax;
		node.aabbMin[axis] = min;
		node.aabbMax[axis] = max;
	}
};