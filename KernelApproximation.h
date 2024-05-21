// @author Merve Asiler

#pragma once

#include "KernelExpansion.h"

/*
* The class which is responsible of conducting the functions of Kernel Approximation algorithm.
*/

class KernelApproximation : public KernelExpansion {

	int numOfLocalGrids = 0;
	queue<int*> possibleVertexProducerIds;

	void expandByExtremeTris();
	void expandBySphereSampling();
	void expandByCornerRays();
	void expandByExactVerts();

	void sendRay(Triangle* triangle, vector<Vertex*>& vertices, double* startPoint, double a, double b, double c);
	void findClosestKernelVertex();
	int findOwnerPlane(double* coords);
	bool isAlreadyKernelVertex(double coords[3]);
	int findIfFrontierObstacle(double* candidateKernelVertex, double* sourcePoint, double* scalarVector);


public:

	KernelApproximation(const Mesh& hostMesh);
	~KernelApproximation();
	void expandKernel();
};