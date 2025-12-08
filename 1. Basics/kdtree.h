#pragma once
#include "accel.h"

namespace Tmpl8 {

class KDTree : public Accel
{
public:
	KDTree() = default;
	KDTree(const char* objFile, uint* objIdxTracker, const float scale = 1) : Accel(objFile, objIdxTracker, scale) {}
	void KDTree::Build()
	{
		for (uint i = 0; i < triCount; i++) 
		{
			// populate triangle index array
			triIdx[i] = i;
			// calculate triangle centroids for partitioning
			tri[i].centroid = (P[tri[i].vertexIdx0] + P[tri[i].vertexIdx1] + P[tri[i].vertexIdx2]) * 0.3333f;
		}
		// assign all triangles to root node
		Node& root = nodes[rootNodeIdx];
		root.leftFirst = 0, root.triCount = triCount;
		// get bounds
		root.aabbMin = float3(1e30f);
		root.aabbMax = float3(-1e30f);
		for (uint first = root.leftFirst, i = 0; i < root.triCount; i++)
		{
			uint leafTriIdx = triIdx[first + i];
			Tri& leafTri = tri[leafTriIdx];
			root.aabbMin = fminf(root.aabbMin, P[leafTri.vertexIdx0]);
			root.aabbMin = fminf(root.aabbMin, P[leafTri.vertexIdx1]);
			root.aabbMin = fminf(root.aabbMin, P[leafTri.vertexIdx2]);
			root.aabbMax = fmaxf(root.aabbMax, P[leafTri.vertexIdx0]);
			root.aabbMax = fmaxf(root.aabbMax, P[leafTri.vertexIdx1]);
			root.aabbMax = fmaxf(root.aabbMax, P[leafTri.vertexIdx2]);
		}
		// subdivide recursively
		Subdivide(rootNodeIdx);
	}

	void KDTree::Intersect(Ray& ray, uint nodeIdx, int* intersectionTests, int* traversalSteps)
	{
		Node* node = &nodes[nodeIdx], * stack[64];
		uint stackPtr = 0;
		if (nodeIdx == rootNodeIdx) // first intersect, transform
		{
			ray.O = TransformPosition(ray.O, invM);
			ray.D = TransformVector(ray.D, invM);
		}
		(*intersectionTests)++;
		(*traversalSteps)++;
		if (IntersectAABB(ray, node->aabbMin, node->aabbMax) == 1e30f) return;
		while (1)
		{
			if (node->isLeaf())
			{
				for (uint i = 0; i < node->triCount; i++)
				{
					IntersectTri(ray, tri[triIdx[node->leftFirst + i]]);
					(*intersectionTests)++;
				}
				if (stackPtr == 0) break; else node = stack[--stackPtr];
				continue;
			}
			Node* child1 = &nodes[node->leftFirst];
			Node* child2 = &nodes[node->leftFirst + 1];

			float dist1 = IntersectAABB(ray, child1->aabbMax, child1->aabbMax);
			float dist2 = IntersectAABB(ray, child2->aabbMax, child2->aabbMax);
			(*intersectionTests)++;
			(*intersectionTests)++;

			if (dist1 > dist2) { swap(dist1, dist2); swap(child1, child2); }
			if (dist1 == 1e30f)
			{
				if (stackPtr == 0) break; else node = stack[--stackPtr];
			}
			else
			{
				node = child1;
				(*traversalSteps)++;
				if (dist2 != 1e30f) stack[stackPtr++] = child2;
			}
		}
		if (nodeIdx == rootNodeIdx) // transform back
		{
			ray.O = TransformPosition(ray.O, M);
			ray.D = TransformVector(ray.D, M);
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
		// handle edge case where triangle n's centroid is outside voxel, but n+1 has vertex inside
		int rightFirst = 0, leftCount = 0;
		for (int l = node.leftFirst; l < node.leftFirst + node.triCount; l++) 
		{
			if (P[tri[triIdx[l]].vertexIdx0][axis] < splitPos ||
				P[tri[triIdx[l]].vertexIdx1][axis] < splitPos ||
				P[tri[triIdx[l]].vertexIdx2][axis] < splitPos) leftCount = l - node.leftFirst + 1;
			if (rightFirst == 0) {
				if (P[tri[triIdx[l]].vertexIdx0][axis] >= splitPos ||
					P[tri[triIdx[l]].vertexIdx1][axis] >= splitPos ||
					P[tri[triIdx[l]].vertexIdx2][axis] >= splitPos) rightFirst = l;
			}
		}

		// abort split if one of the sides is empty or if every triangle intersects both sides
		if (leftCount == 0 || leftCount == node.triCount) return;
		// create child nodes
		int leftChildIdx = nodesUsed++;
		int rightChildIdx = nodesUsed++;
		nodes[leftChildIdx].leftFirst = node.leftFirst;
		nodes[leftChildIdx].triCount = leftCount;
		nodes[rightChildIdx].leftFirst = rightFirst;
		nodes[rightChildIdx].triCount = node.leftFirst + node.triCount - rightFirst;
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

}