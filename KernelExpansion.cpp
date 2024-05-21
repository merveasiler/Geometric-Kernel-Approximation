// @author Merve Asiler

#include "KernelExpansion.h"
#include "BaseGeoOpUtils.h"
#include "sdlp.h"

/*
* @abstract Finds the extreme kernel points in the six principal directions(+x, -x, +y, -y, +z, -z).
			Constructs the AABB (the axis aligned bounding box) of the kernel.
			Defines the bounding halfspaces using triangles of the input mesh.
*/
KernelExpansion::KernelExpansion(const Mesh& hostMesh) {

	this->hostMeshptr = &hostMesh;
	
	// extract the halfspace inequalites using the input mesh triangles
	computeHalfSpacesFromTriangles(hostMeshptr->getAllTris(), hostMeshptr->getAllVerts(), this->halfSpaceSet);

	// Construct the AABB (Axis-Aligned Bounding Box) of the kernel
	// Find the extreme kernel points in the 6 main directions 
	extremeCorners[0] = new double[3]{ 0, 0, 0 };	extremeCorners[1] = new double[3]{ 0, 0, 0 };
	double extremeDirections[6][3] = { {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1} };
	for (int i = 0; i < 6; i++) {
		this->extremePoints[i] = nullptr;
		this->extremePoints[i] = sdlpMain(extremeDirections[i], halfSpaceSet);	// compute the kernel point at the given extreme direction
		if (!this->extremePoints[i]) {	// kernel is empty
			is_kernel_empty = true;
			return;
		}
		// compute the cross corners of the AABB step-by-step (the corners in the directions (-x,-y,-z) and (x,y,z) 
		this->extremeCorners[i % 2][int(i / 2)] = this->extremePoints[i][int(i / 2)];
	}

	// compute the center of the AABB
	for (int i = 0; i < 3; i++) 
		this->AABB_center[i] = (this->extremeCorners[0][i] + this->extremeCorners[1][i]) / 2.0;	
	
}

KernelExpansion::~KernelExpansion() {

	// deletes, clear ups

	if (extremeCorners[0]) {
		delete[] extremeCorners[0];
		extremeCorners[0] = nullptr;
		delete[] extremeCorners[1];
		extremeCorners[1] = nullptr;
	}

	for (int i = 0; i < 6; i++) {
		if (extremePoints[i]) {
			delete[] extremePoints[i];
			extremePoints[i] = nullptr;
		}
	}

	halfSpaceSet.clear();

}

Mesh& KernelExpansion::getKernel() {
	return kernel;
}

vector<HalfSpace>& KernelExpansion::getHalfSpaceSet() {
	return halfSpaceSet;
}

bool KernelExpansion::isInKernel(double* scalarVector) {
	for (int i = 0; i < halfSpaceSet.size(); i++)
		if (scalarVector[i] > EPSILON_3)
			return false;	// out
	return true;		// in
}

vector<int> KernelExpansion::detectExcluderHalfSpaces(double* scalarVector) {
	vector<int> excluderIds;
	for (int i = 0; i < halfSpaceSet.size(); i++)
		if (scalarVector[i] > EPSILON_3)
			excluderIds.push_back(i);
	return excluderIds;
}

void KernelExpansion::checkKernelForNonKernelVertices() {

	int num_of_non_kernel_points = 0;
	for (int i = 0; i < kernel.getNumOfVerts(); i++) {
		if (findClosestValueSatisfiedByPoint(kernel.getVertex(i).coords, halfSpaceSet) > 3 * EPSILON ||
			findClosestValueSatisfiedByPoint(kernel.getVertex(i).coords, halfSpaceSet) > 3 * EPSILON ||
			findClosestValueSatisfiedByPoint(kernel.getVertex(i).coords, halfSpaceSet) > 3 * EPSILON) {
			cout << "non-kernel point" << endl;
			num_of_non_kernel_points++;
		}
	}
	cout << "num of non-kernel points is: " << num_of_non_kernel_points << endl;

}

void KernelExpansion::checkKernelForIrregularTriangles() {

	int num_of_irregular_triangles = 0;
	for (int i = 0; i < kernel.getNumOfTris(); i++) {
		if (kernel.getTriangle(i).corners[0] == kernel.getTriangle(i).corners[1] ||
			kernel.getTriangle(i).corners[1] == kernel.getTriangle(i).corners[2] ||
			kernel.getTriangle(i).corners[2] == kernel.getTriangle(i).corners[0]) {
			cout << "errorrrrr: triangle no: " << i << endl;
			num_of_irregular_triangles++;
		}
	}
	cout << "num of irregular triangles is: " << num_of_irregular_triangles << endl;

}

