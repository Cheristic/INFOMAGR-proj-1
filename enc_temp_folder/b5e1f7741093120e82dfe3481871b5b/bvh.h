#pragma once
#include "accel.h"
#include "tmplmath.h"

namespace Tmpl8 {

class BVH : public Accel
{
public:
	BVH(const char* objFile, uint* objIdxTracker, const float scale = 1) : Accel(objFile, objIdxTracker, scale) {}
	void BVH::Build() // CHANGE TO SET TRI COUNT TO MODEL TRI COUNT
	{
		for (uint i = 0; i < triCount; i++) {
			// populate triangle index array
			triIdx[i] = i;
			// calculate triangle centroids for partitioning
			tri[i].centroid = (tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2) * 0.3333f;
		}
		cout << triCount;
		// assign all triangles to root node
		Node& root = nodes[rootNodeIdx];
		root.leftFirst = 0, root.triCount = triCount;
		UpdateNodeBounds(rootNodeIdx);
		// subdivide recursively
		Subdivide(rootNodeIdx);
	}
	void BVH::Intersect(Ray& ray, uint nodeIdx)
	{
		Node* node = &nodes[nodeIdx], * stack[64];
		uint stackPtr = 0;

		while (1)
		{
			//if (IntersectAABB(ray, node->aabbMin, node->aabbMax) == 1e30f) return;
			if (node->isLeaf())
			{
				for (uint i = 0; i < node->triCount; i++)
					IntersectTri(ray, tri[triIdx[node->leftFirst + i]]);

				if (stackPtr == 0) break; else node = stack[--stackPtr];
			}
			Node* child1 = &nodes[node->leftFirst];
			Node* child2 = &nodes[node->leftFirst + 1];

			float dist1 = IntersectAABB(ray, child1->aabbMin, child1->aabbMax);
			float dist2 = IntersectAABB(ray, child2->aabbMin, child2->aabbMax);

			if (dist1 > dist2) { swap(dist1, dist2); swap(child1, child2); }
			if (dist1 == 1e30f)
			{
				if (stackPtr == 0) break; else node = stack[--stackPtr];
			}
			else
			{
				node = child1;
				if (dist2 != 1e30f) stack[stackPtr++] = child2;
			}
		}
	}
	void BVH::Subdivide(uint nodeIdx)
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
		UpdateNodeBounds(leftChildIdx);
		UpdateNodeBounds(rightChildIdx);
		// recurse
		Subdivide(leftChildIdx);
		Subdivide(rightChildIdx);
	}

	void BVH::UpdateNodeBounds(uint nodeIdx)
	{
		Node& node = nodes[nodeIdx];
		node.aabbMin = float3(1e30f);
		node.aabbMax = float3(-1e30f);
		for (uint first = node.leftFirst, i = 0; i < node.triCount; i++)
		{
			uint leafTriIdx = triIdx[first + i];
			Tri& leafTri = tri[leafTriIdx];
			node.aabbMin = fminf(node.aabbMin, leafTri.vertex0);
			node.aabbMin = fminf(node.aabbMin, leafTri.vertex1);
			node.aabbMin = fminf(node.aabbMin, leafTri.vertex2);
			node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex0);
			node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex1);
			node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex2);
		}
	}

};

}