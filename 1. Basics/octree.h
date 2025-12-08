#pragma once
#include "accel.h"
#include "tmplmath.h"
//#include "objects.h"

namespace Tmpl8 {

class Octree : public Accel
{
public:
	Octree() = default;
	Octree(const char* objFile, uint* objIdxTracker, const float scale = 1, float3 offset = 0) : Accel(objFile, objIdxTracker, scale, offset) {}

	void Octree::Build()
	{
		//nodes = new Node[triCount * 4];
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

	void Octree::Subdivide(uint nodeIdx)
	{
		Node& node = nodes[nodeIdx];
		float3 splitPos;
		float bestCost = 1e30f;
		for (int axis = 0; axis < 3; axis++)
		{
			for (uint i = 0; i < node.triCount; i++)
			{
				Tri& triangle = tri[triIdx[node.leftFirst + i]];
				float candidatePos = triangle.centroid[axis];
				float cost = 100; // EvaluateSAH(node, axis, candidatePos);
				if (cost < bestCost)
				{
					splitPos[axis] = candidatePos, bestCost = cost;
				}
			}
		}
		float3 e = node.aabbMax - node.aabbMin;
		float parentArea = e.x * e.x + e.y * e.y + e.z * e.z;
		float parentCost = node.triCount * parentArea;
		if (bestCost >= parentCost) return;

		// Organize triangles for first octant
		int leftFirstArray[9];
		leftFirstArray[0] = node.leftFirst;
		ArrangeTriangles(node, splitPos, leftFirstArray, 1);
		ArrangeTriangles(node, splitPos, leftFirstArray, 2);
		ArrangeTriangles(node, splitPos, leftFirstArray, 3);
		ArrangeTriangles(node, splitPos, leftFirstArray, 4);
		ArrangeTriangles(node, splitPos, leftFirstArray, 5);
		ArrangeTriangles(node, splitPos, leftFirstArray, 6);
		ArrangeTriangles(node, splitPos, leftFirstArray, 7);
		ArrangeTriangles(node, splitPos, leftFirstArray, 8);

		// abort split if at least 4 octants are empty
		int emptyCount = 0;
		if (leftFirstArray[1] - node.triCount == 0) emptyCount++;
		for (int i = 2; i <= 8; i++)
		{
			if (leftFirstArray[i] - leftFirstArray[i - 1] == 0) emptyCount++;
		}
		if (emptyCount >= 4) return;

		// Create child nodes
		int firstChildIdx = nodesUsed++;
		nodes[firstChildIdx].leftFirst = node.leftFirst;
		nodes[firstChildIdx].triCount = leftFirstArray[1] - node.leftFirst;
		for (int i = 2; i <= 8; i++)
		{
			int childIdx = nodesUsed++;
			nodes[childIdx].leftFirst = leftFirstArray[i];
			nodes[childIdx].triCount = leftFirstArray[i] - leftFirstArray[i - 1];
		}
		node.leftFirst = firstChildIdx;
		node.triCount = 0;

		UpdateNodeBounds(firstChildIdx, node.aabbMin, splitPos);
		UpdateNodeBounds(firstChildIdx + 1, float3(node.aabbMin[0], node.aabbMin[1], splitPos[2]), float3(splitPos[0], splitPos[1], node.aabbMax[2]));
		UpdateNodeBounds(firstChildIdx + 2, float3(node.aabbMin[0], splitPos[1], node.aabbMin[2]), float3(splitPos[0], node.aabbMax[1], splitPos[2]));
		UpdateNodeBounds(firstChildIdx + 3, float3(node.aabbMin[0], splitPos[1], splitPos[2]), float3(splitPos[0], node.aabbMax[1], node.aabbMax[2]));
		UpdateNodeBounds(firstChildIdx + 4, float3(splitPos[0], node.aabbMin[1], node.aabbMin[2]), float3(node.aabbMax[0], splitPos[1], splitPos[2]));
		UpdateNodeBounds(firstChildIdx + 5, float3(splitPos[0], node.aabbMin[1], splitPos[2]), float3(node.aabbMax[0], splitPos[1], node.aabbMax[2]));
		UpdateNodeBounds(firstChildIdx + 6, float3(splitPos[0], splitPos[1], node.aabbMin[2]), float3(node.aabbMax[0], node.aabbMax[1], splitPos[2]));
		UpdateNodeBounds(firstChildIdx + 7, splitPos, node.aabbMax);

		for (int i = 0; i < 8; i++)
		{
			Subdivide(firstChildIdx + i);
		}
	}

	void Octree::Intersect(Ray& ray, uint nodeIdx, int* intersectionTests, int* traversalSteps)
	{
		Node* node = &nodes[nodeIdx], * stack[64];
		uint stackPtr = 0;

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
			Node* child[8];
			child[0] = &nodes[node->leftFirst];
			child[1] = &nodes[node->leftFirst + 1];
			child[2] = &nodes[node->leftFirst + 2];
			child[3] = &nodes[node->leftFirst + 3];
			child[4] = &nodes[node->leftFirst + 4];
			child[5] = &nodes[node->leftFirst + 5];
			child[6] = &nodes[node->leftFirst + 6];
			child[7] = &nodes[node->leftFirst + 7];

			float dist[8];
			dist[0] = IntersectAABB(ray, child[0]->aabbMax, child[0]->aabbMax);
			dist[1] = IntersectAABB(ray, child[1]->aabbMax, child[1]->aabbMax);
			dist[2] = IntersectAABB(ray, child[2]->aabbMax, child[2]->aabbMax);
			dist[3] = IntersectAABB(ray, child[3]->aabbMax, child[3]->aabbMax);
			dist[4] = IntersectAABB(ray, child[4]->aabbMax, child[4]->aabbMax);
			dist[5] = IntersectAABB(ray, child[5]->aabbMax, child[5]->aabbMax);
			dist[6] = IntersectAABB(ray, child[6]->aabbMax, child[6]->aabbMax);
			dist[7] = IntersectAABB(ray, child[7]->aabbMax, child[7]->aabbMax);
			(*intersectionTests) += 8;

			for (int i = 0; i < 8; i++) for (int j = 0; j < 8; j++)
			{
				if (dist[i] > dist[j]) { swap(dist[i], dist[j]); swap(child[i], child[j]); }
			}
			
			if (dist[0] == 1e30f)
			{
				if (stackPtr == 0) break; else node = stack[--stackPtr];
			}
			else
			{
				node = child[0];
				(*traversalSteps)++;
				if (dist[1] != 1e30f) stack[stackPtr++] = child[1];
				if (dist[2] != 1e30f) stack[stackPtr++] = child[2];
				if (dist[3] != 1e30f) stack[stackPtr++] = child[3];
				if (dist[4] != 1e30f) stack[stackPtr++] = child[4];
				if (dist[5] != 1e30f) stack[stackPtr++] = child[5];
				if (dist[6] != 1e30f) stack[stackPtr++] = child[6];
				if (dist[7] != 1e30f) stack[stackPtr++] = child[7];

			}
		}

	}

	void Octree::ArrangeTriangles(const Node& node, float3 splitPos, int (&leftFirstArray)[9], uint octant)
	{
		int i = leftFirstArray[0];
		int j = i + node.triCount - 1;
		while (i <= j)
		{
			Tri& t = tri[triIdx[i]];
			switch (octant)
			{
			case 1:
				if (t.centroid[0] < splitPos[0] && t.centroid[1] < splitPos[1] && t.centroid[2] < splitPos[2]) goto Add;
			case 2:
				if (t.centroid[0] < splitPos[0] && t.centroid[1] < splitPos[1] && t.centroid[2] >= splitPos[2]) goto Add;
			case 3:
				if (t.centroid[0] < splitPos[0] && t.centroid[1] >= splitPos[1] && t.centroid[2] < splitPos[2]) goto Add;
			case 4:
				if (t.centroid[0] < splitPos[0] && t.centroid[1] >= splitPos[1] && t.centroid[2] >= splitPos[2]) goto Add;
			case 5:
				if (t.centroid[0] >= splitPos[0] && t.centroid[1] < splitPos[1] && t.centroid[2] < splitPos[2]) goto Add;
			case 6:
				if (t.centroid[0] >= splitPos[0] && t.centroid[1] < splitPos[1] && t.centroid[2] >= splitPos[2]) goto Add;
			case 7:
				if (t.centroid[0] >= splitPos[0] && t.centroid[1] >= splitPos[1] && t.centroid[2] < splitPos[2]) goto Add;
			case 8:
				if (t.centroid[0] >= splitPos[0] && t.centroid[1] >= splitPos[1] && t.centroid[2] >= splitPos[2]) goto Add;
			default:
				return;
			}
			swap(triIdx[i], triIdx[j--]);
			continue;
		Add:
			++i;
			++leftFirstArray[octant];
		}
	}

	void Octree::UpdateNodeBounds(uint nodeIdx, const float3& min, const float3& max)
	{
		Node& node = nodes[nodeIdx];
		node.aabbMin = min;
		node.aabbMax = max;
	}
};
}