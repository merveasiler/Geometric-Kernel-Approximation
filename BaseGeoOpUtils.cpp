// @author Merve Asiler

#include "BaseGeoOpUtils.h"
#include <iostream>
#include <Eigen/Dense>

int findPeakVertexID(Triangle* triangle, Edge* edge) {
	for (unsigned int i = 0; i < 3; i++) {
		if (triangle->corners[i] == edge->endVerts[0] || triangle->corners[i] == edge->endVerts[1])
			continue;
		return triangle->corners[i];
	}
	return INVALID_TRIANGLE;	// "not found" is not a possible case

}

double* findLine2LineIntersection(Line* line1, Line* line2) {	// Assume that they have a certain intersection point
	Eigen::Matrix2d A;
	A << line2->directionVector[0], -line1->directionVector[0],
		line2->directionVector[1], -line1->directionVector[1];
	Eigen::Vector2d b;
	b << line1->point[0] - line2->point[0],
		line1->point[1] - line2->point[1];
	Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

	double* point = new double[3];
	for (int i = 0; i < 3; i++)
		point[i] = line1->point[i] + line1->directionVector[i] * x[1];
	return point;
}

double findLinePlaneIntersection(Line& line, Plane& plane) {
	/* Assume they intersect at q = p + v*t (the rhs is parametric line equation)
	   A * q.x + B * q.y + C * q.z + D = 0	(comes from the plane equation)
	*/
	
	double midProduct = dotProduct(plane.ABCD, line.point);
	double numerator = midProduct + plane.ABCD[3];
	double denominator = dotProduct(plane.ABCD, line.directionVector);
	
	if (abs(denominator) < 3 * EPSILON)
		return numeric_limits<double>::infinity();
	return -numerator / denominator;
}

double findRayTriangleIntersection(Line& line, Triangle& triangle, Plane& trianglePlane) {	// assumes the triangle's normal is precomputed

	double coeff = findLinePlaneIntersection(line, trianglePlane);

	if (isinf(coeff))
		return numeric_limits<double>::infinity();

	double Q[3];	// intersection point
	for (int k = 0; k < 3; k++)
		Q[k] = line.point[k] + coeff * line.directionVector[k];

	// compute barycentric coordinates of Q
	double* baryCoords = triangle.computeBarycentricCoords(Q, trianglePlane.point1, trianglePlane.point2, trianglePlane.point3);
	if (baryCoords == NULL)
		return numeric_limits<double>::infinity();

	delete[] baryCoords;
	return coeff;

}

double findRayTriangleIntersection(Line* ray, TriangleWithVerts* triangleWV) {

	triangleWV->computeNormal(triangleWV->vertices[0], triangleWV->vertices[1], triangleWV->vertices[2]);
	Plane* plane = new Plane(triangleWV->vertices[0]->coords, triangleWV->normal);

	double coeff = findLinePlaneIntersection(*ray, *plane);
	delete plane;
	if (isinf(coeff)) {
		return numeric_limits<double>::infinity();
	}

	double Q[3];	// intersection point
	for (int k = 0; k < 3; k++)
		Q[k] = ray->point[k] + coeff * ray->directionVector[k];

	// compute barycentric coordinates of Q
	double* baryCoords = triangleWV->computeBarycentricCoords(Q);
	if (baryCoords == NULL)
		return numeric_limits<double>::infinity();
	delete[] baryCoords;
	return coeff;

}

Line* find2PlaneIntersection(Plane& plane1, Plane& plane2) {

	double cosAngle = dotProduct(plane1.ABCD, plane2.ABCD);
	if (fabs(fabs(cosAngle) - 1) < EPSILON)
		return NULL;

	double dirVector[3];
	crossProduct(plane1.ABCD, plane2.ABCD, dirVector);
	normalize(dirVector);

	bool point_found = false;
	double point[3];
	double coeffs[2][4] = { {plane1.ABCD[0], plane1.ABCD[1], plane1.ABCD[2], plane1.ABCD[3]}, {plane2.ABCD[0], plane2.ABCD[1], plane2.ABCD[2], plane2.ABCD[3]} };

	for (int i = 0; i < 3; i++) {

		double det = coeffs[0][i] * coeffs[1][(i + 1) % 3] - coeffs[0][(i + 1) % 3] * coeffs[1][i];
		if (abs(det) < 2 * EPSILON)
			continue;

		point[i] = (-coeffs[1][(i + 1) % 3] * coeffs[0][3] + coeffs[0][(i + 1) % 3] * coeffs[1][3]) / det;
		if (abs(coeffs[0][(i + 1) % 3]) < EPSILON)
			point[(i + 1) % 3] = (-coeffs[1][3] - point[i] * coeffs[1][i]) / coeffs[1][(i + 1) % 3];
		else
			point[(i + 1) % 3] = (-coeffs[0][3] - point[i] * coeffs[0][i]) / coeffs[0][(i + 1) % 3];
		point[(i + 2) % 3] = 0;

		point_found = true;
		break;
	}

	if (point_found)
		return new Line(point, dirVector);
	return NULL;

}

double* find3PlaneIntersection(Plane& plane1, Plane& plane2, Plane& plane3) {

	Line* line = find2PlaneIntersection(plane1, plane2);
	if (line == NULL)
		return NULL;

	double t = findLinePlaneIntersection(*line, plane3);
	
	if (t == numeric_limits<double>::infinity()) {
		delete line;
		return NULL;
	}
	
	double* point = new double[3];
	for (int i = 0; i < 3; i++)
		point[i] = line->directionVector[i] * t + line->point[i];
	
	delete line;
	return point;

	/*
	// Cramer's rule

	double ABC[3][3] = { {planes[0].ABCD[0], planes[0].ABCD[1], planes[0].ABCD[2]},
						 {planes[1].ABCD[0], planes[1].ABCD[1], planes[1].ABCD[2]},
						 {planes[2].ABCD[0], planes[2].ABCD[1], planes[2].ABCD[2]} };
	
	double detABC = computeDeterminant3x3(ABC);
	if (abs(detABC) < 3*EPSILON*EPSILON)
		return NULL;

	double ABC_x[3][3], ABC_y[3][3], ABC_z[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			ABC_x[i][j] = ABC[i][j];
			ABC_y[i][j] = ABC[i][j];
			ABC_z[i][j] = ABC[i][j];
		}
		ABC_x[i][0] = -planes[i].ABCD[3];
		ABC_y[i][1] = -planes[i].ABCD[3];
		ABC_z[i][2] = -planes[i].ABCD[3];
	}

	double* xyz = new double[3];
	double dets[3] = { computeDeterminant3x3(ABC_x), computeDeterminant3x3(ABC_y), computeDeterminant3x3(ABC_z) };

	for (int i = 0; i < 3; i++)
		xyz[i] = dets[i] / detABC;

	return xyz;
	*/
}

double* projectVertexOnPlane(Plane* plane, double* vertex) {
	double* point = new double[3];
	double A = plane->ABCD[0];
	double B = plane->ABCD[1];
	double C = plane->ABCD[2];
	double D = plane->ABCD[3];
	double Px = vertex[0];
	double Py = vertex[1];
	double Pz = vertex[2];

	/* If the projection coordinates are Q=(x,y,z),
		then P-Q = t*N, where N is the plane normal.
		In other words:
		(x - Px, y - Py, z - Pz) = t * (Nx, Ny, Nz)
		This results in 3 equations:
		x = Px + t * Nx	
		y = Py + t * Ny
		z = Pz + t * Nz
		Put Q=(xy,z) into the plane equation:
		A * (Px + t * Nx) + B * (Py + t * Ny) + C * (Pz + t * Nz) + D = 0
		=> t = -(A*Px + B*Py + C*Pz + D) / (A*Nx + B*Ny + C*Nz)
		Do not forget that <Nx, Ny, Nz> = <A, B, C>.
	*/
	double t = -(A * Px + B * Py + C * Pz + D) / (A * A + B * B + C * C);
	for (int i = 0; i < 3; i++)
		point[i] = vertex[i] + t * plane->ABCD[i];

	/*
	// TO-DO: What if either one of A,B,C is 0?
	double tempProductY = (A * Py - B * Px);
	double tempProductZ = (A * Pz - C * Px);

	point[0] = -(D * A + (B * tempProductY + C * tempProductZ)) / (A * A + B * B + C * C);
	point[1] = (tempProductY + B * point[0]) / A;
	point[2] = (tempProductZ + C * point[0]) / A;
	*/
	return point;
}

/*
	Is the given 3D point inside the region enclosed by the given half space?
	returns 1 if the point is inside the region
	returns 0 if the point is on the boundary of the region
	returns -1 if the point is outside the region
*/
int isPointInRegion(const double* point, const HalfSpace* halfSpace) {
	double total = halfSpace->ABCD[3];
	for (unsigned int j = 0; j < 3; j++)
		total += halfSpace->ABCD[j] * point[j];
	if (abs(total) < EPSILON)
		return 0;	// on the boundary
	else if (halfSpace->isLargerThanZero && total >= 0)
		return 1;	// inside
	else if (!halfSpace->isLargerThanZero && total <= 0)
		return 1;	// inside
	else
		return -1;	// outside
}

double findClosestValueSatisfiedByPoint(const double point[3], const vector<HalfSpace>& halfSpaceContainer) {

	double closestValue = numeric_limits<double>::infinity();
	double furthestValue = -numeric_limits<double>::infinity();

	for (unsigned int i = 0; i < halfSpaceContainer.size(); i++) {
		HalfSpace halfSpace = halfSpaceContainer[i];
		
		double total = halfSpace.ABCD[3];
		for (unsigned int j = 0; j < 3; j++) {
			double mult = halfSpace.ABCD[j] * point[j];
			total += mult;
		}
		
		if ( (halfSpace.isLargerThanZero && total >= 0) || (!halfSpace.isLargerThanZero && total <= 0) ) {
			if (abs(total) - closestValue < 3*EPSILON)
				closestValue = abs(total);
		}
		else {
			if (furthestValue - abs(total) < 3*EPSILON)
				furthestValue = abs(total);
		}
	}

	if (furthestValue > 0)
		return furthestValue;
	return -closestValue;
}

double* findValueVectorSatisfiedByPoint(const double point[3], const vector<HalfSpace>& halfSpaceContainer) {
	
	double* valuesVector = new double[halfSpaceContainer.size()];

	for (unsigned int i = 0; i < halfSpaceContainer.size(); i++) {

		double total = halfSpaceContainer[i].ABCD[3];
		for (unsigned int j = 0; j < 3; j++) {
			double mult = halfSpaceContainer[i].ABCD[j] * point[j];
			total += mult;
		}

		valuesVector[i] = total;
	}

	return valuesVector;
}

double isPointInHalfSpace(HalfSpace& halfspace, double* point) {

	double distance = halfspace.ABCD[3];
	for (unsigned int j = 0; j < 3; j++) {
		double mult = halfspace.ABCD[j] * point[j];
		distance += mult;
	}

	return distance;

}

void computeHalfSpacesFromTriangles(const vector<Triangle>& tris, const vector<Vertex>& verts, vector<HalfSpace>& halfSpaces) {
	
	for (int i = 0; i < tris.size(); i++) {
		Triangle tri = tris[i];
		HalfSpace halfSpace(verts[tri.corners[0]].coords, tri.normal, false);
		halfSpaces.push_back(halfSpace);
	}

}

vector<double*> computeHalfSpaceCoeffsFromTriangles(const vector<Triangle>& tris, const vector<Vertex>& verts) {

	vector<double*> halfSpaceCoeffs;
	vector<HalfSpace> halfSpaces;
	computeHalfSpacesFromTriangles(tris, verts, halfSpaces);

	for (int i = 0; i < halfSpaces.size(); i++) {
		double* coeffs = new double[4];
		for (int j = 0; j < 4; j++)
			coeffs[j] = halfSpaces[i].ABCD[j];
		halfSpaceCoeffs.push_back(coeffs);
	}
	halfSpaces.clear();
	
	return halfSpaceCoeffs;
}

double findAngleBetweenPlaneAndVector(const Plane* plane, const double* direction) {

	double cosAngleWithNormal = findCosAngleBetween(plane->ABCD, direction);
	double angleWithNormal = acos(cosAngleWithNormal);
	return abs(angleWithNormal - M_PI_2);

}

double* projectVectorOnPlane(const Plane* plane, const double* direction) {

	double q[3];	// represents the point obtained by going along the <direction> from <plane->point>
	for (int i = 0; i < 3; i++)
		q[i] = plane->point[i] + direction[i];
	
	double angle = findAngleBetweenPlaneAndVector(plane, direction);
	double height = computeLength(direction) * abs(sin(angle));	// height of q to the plane
	
	double p[3]; // projection of q on to the plane
	for (int i = 0; i < 3; i++)
		p[i] = q[i] - plane->ABCD[i] * height;

	double* projection = new double[3];	// projection of <direction> on to the <plane>
	for (int i = 0; i < 3; i++)
		projection[i] = p[i] - plane->point[i];

	return projection;
}

HalfPlane* projectHalfSpaceOnPlane(const Plane* basePlane, const HalfSpace* hs) {

	double* directionOfLine = crossProduct(basePlane->ABCD, hs->ABCD);	// direction of the line occuring with the intersection of two planes 
	// Check whether the two planes are parallel (or the same) or not
	bool is_parallel = true;
	for (int i = 0; i < 3; i++)	// if directionOfLine is <0, 0, 0>
		if (abs(directionOfLine[i]) > EPSILON)
			is_parallel = false;
	if (is_parallel)
		throw PARALLEL_PLANE_EXCEPTION();

	// Find a common point of two planes:
	Eigen::MatrixXd A(2,3);
	A << basePlane->ABCD[0], basePlane->ABCD[1], basePlane->ABCD[2],
		 hs->ABCD[0], hs->ABCD[1], hs->ABCD[2];
	Eigen::Vector2d b;
	b << -basePlane->ABCD[3], -hs->ABCD[3];
	Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
	double point[3] = {x[0], x[1], x[2]};

	// lastly, compute the normal of half plane (on the basePlane) and construct it
	double* normalOnPlane = projectVectorOnPlane(basePlane, hs->ABCD);
	normalize(normalOnPlane);
	HalfPlane* projection = new HalfPlane(point, directionOfLine, normalOnPlane, false);

	// clean-up
	delete[] directionOfLine;
	directionOfLine = nullptr;
	delete[] normalOnPlane;
	normalOnPlane = nullptr;

	return projection;

}

double* rotate3x1(double rotationMatrix[3][3], double element3x1[3]) {
	double* rotated3x1 = new double[3];
	for (int i = 0; i < 3; i++) {
		rotated3x1[i] = 0;
		for (int j = 0; j < 3; j++)
			rotated3x1[i] += rotationMatrix[i][j] * element3x1[j];
	}
	return rotated3x1;
}

bool isZeroVector(double* vect) {
	bool result = true;
	for (int i = 0; i < 3; i++)
		result = result && (abs(vect[i]) < EPSILON ? true : false);
	return result;
}

bool isTripleSame(double* p1, double* p2) {

	if (abs(p1[0] - p2[0]) < EPSILON && abs(p1[1] - p2[1]) < EPSILON && abs(p1[2] - p2[2]) < EPSILON)
		return true;
	return false;
}

double** computeBoxCoords(double minCoords[3], double maxCoords[3]) {
	
	double** coords = new double* [8];
	// ---> handle the corners in the ordering taken by first change in x, then in z, then in y
	coords[0] = new double[3]{ minCoords[0], minCoords[1], minCoords[2] };
	coords[1] = new double[3]{ maxCoords[0], minCoords[1], minCoords[2] };
	coords[2] = new double[3]{ maxCoords[0], minCoords[1], maxCoords[2] };
	coords[3] = new double[3]{ minCoords[0], minCoords[1], maxCoords[2] };
	coords[4] = new double[3]{ minCoords[0], maxCoords[1], minCoords[2] };
	coords[5] = new double[3]{ maxCoords[0], maxCoords[1], minCoords[2] };
	coords[6] = new double[3]{ maxCoords[0], maxCoords[1], maxCoords[2] };
	coords[7] = new double[3]{ minCoords[0], maxCoords[1], maxCoords[2] };

	return coords;
}


