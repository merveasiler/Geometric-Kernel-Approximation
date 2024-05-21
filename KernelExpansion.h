// @author Merve Asiler

#pragma once

#include "Mesh.h"
#include "BasicGeometricElements.h"
#include <queue>

/*
* The abstract class from which kernel computation algorithms (CGAL and the proposed Kernel Approximation algorithm) are derived.
*/

class KernelExpansion {

protected:
	bool is_kernel_empty = false;
	double AABB_center[3] = { 0, 0, 0 };
	double* extremePoints[6] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
	const Mesh* hostMeshptr;
	Mesh kernel;
	vector<HalfSpace> halfSpaceSet;
	double EPSILON_3 = 3 * EPSILON;

	bool isInKernel(double* scalarVector);
	vector<int> detectExcluderHalfSpaces(double* scalarVector);

public:
	double* extremeCorners[2] = { nullptr, nullptr };
	KernelExpansion(const Mesh& hostMesh);
	~KernelExpansion();
	Mesh& getKernel();
	vector<HalfSpace>& getHalfSpaceSet();
	virtual void expandKernel() = 0;

	void checkKernelForNonKernelVertices();
	void checkKernelForIrregularTriangles();
};
