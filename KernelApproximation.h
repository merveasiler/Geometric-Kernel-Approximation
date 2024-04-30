// @author Merve Asiler

#pragma once

#include "KernelExpansion.h"

class KernelApproximation : public KernelExpansion {

	int numOfLocalGrids = 0;
	queue<int*> possibleVertexProducerIds;
	int num_of_rays_per_tris = 29;
	int recursion_depth = -2;

	void expandByExtremeTris();
	void expandByRegularTris();
	void expandBySphereSampling();
	void expandByCornerRays();
	void expandByExactVerts();

	void sendRay(Triangle* triangle, vector<Vertex*>& vertices, double* startPoint, double a, double b, double c);
	void findClosestKernelVertex(double source_coords[3]);
	int findOwnerPlane(double* coords);
	bool isAlreadyKernelVertex(double coords[3]);
	int findIfFrontierObstacle(double* candidateKernelVertex, double* sourcePoint, int candidateVertexProducerIds[3], double* scalarVector);


public:
	KernelApproximation(const Mesh& hostMesh);
	~KernelApproximation();
	void expandKernel();
};