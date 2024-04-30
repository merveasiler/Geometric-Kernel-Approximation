// @author Merve Asiler

#include "KernelApproximation.h"
#include "BaseGeoOpUtils.h"
#include "CGALUtils.h"

#include <random>

KernelApproximation::KernelApproximation(const Mesh& hostMesh) :
	
	KernelExpansion(hostMesh, true) {

};

KernelApproximation::~KernelApproximation() {


}

void KernelApproximation::expandKernel() {

	// is kernel empty?
	if (this->initialPoint == nullptr)
		return;	// NOT STAR-SHAPED!

	expandByExtremeTris();

	//expandBySphereSampling();

	expandByCornerRays();

	kernel = computeConvexHull(kernel.getAllVerts());

	expandByExactVerts();

	vector <Vertex> filteredVertices;
	for (int v = 0; v < kernel.getNumOfVerts(); v++) {
		double* scalars = findValueVectorSatisfiedByPoint(kernel.getVertex(v).coords, halfSpaceSet);
		if (isInKernel(scalars))
			filteredVertices.push_back(kernel.getVertex(v));
		delete[] scalars;
	}

	kernel = computeConvexHull(filteredVertices);

}

void KernelApproximation::expandByExtremeTris() {

	double AABB_center[3] = { 0, 0, 0 };
	for (int n = 0; n < 6; n++)
		for (int d = 0; d < 3; d++)
			AABB_center[d] += extremePoints[n][d];
	for (int d = 0; d < 3; d++)
		AABB_center[d] /= 6.0;

	// Define triangles in which rays pass through
	vector<Triangle*> triangles;
	vector<Vertex*> vertices;
	vector<double> trisAreas;

	int t = 0;
	for (int i = 0; i < 2; i++) {
		for (int j = 2; j < 4; j++) {
			for (int k = 4; k < 6; k++) {
				int vertex_ids[3] = { t * 3, t * 3 + 1, t * 3 + 2 };
				vertices.push_back(new Vertex(vertex_ids[0], extremePoints[i]));
				vertices.push_back(new Vertex(vertex_ids[1], extremePoints[j]));
				vertices.push_back(new Vertex(vertex_ids[2], extremePoints[k]));

				Triangle* triangle = new Triangle(t, vertex_ids);
				triangles.push_back(triangle);

				double* areaVector = triangle->computeAreaVector(vertices[t * 3], vertices[t * 3 + 1], vertices[t * 3 + 2]);
				trisAreas.push_back(computeLength(areaVector));
				delete[] areaVector;
				t++;
			}
		}
	}

	/*
	// normalize triagle areas
	double maxArea = 0;
	for (int t = 0; t < 8; t++) {
		if (trisAreas[t] > maxArea)
			maxArea = trisAreas[t];
	}
	for (int t = 0; t < 8; t++) {
		if (trisAreas[t] < EPSILON) {
			double maxEdgeLength = 0;
			for (int v = 0; v < 3; v++) {
				double* edgeVector = diffVects(vertices[3 * t + v]->coords, vertices[3 * t + (v + 1) % 3]->coords);
				double edgeLength = computeLength(edgeVector);
				delete[] edgeVector;
				if (edgeLength > maxEdgeLength)
					maxEdgeLength = edgeLength;
			}
			trisAreas[t] = maxEdgeLength;
		}
		trisAreas[t] /= maxArea;
	}
	*/

	t = 0;
	for (int i = 0; i < 2; i++) {
		for (int j = 2; j < 4; j++) {
			for (int k = 4; k < 6; k++) {

				Triangle* triangle = triangles[t];
				double* startPoint = AABB_center;
				t++;

				std::random_device rd;  // Will be used to obtain a seed for the random number engine
				std::mt19937 gen(19937); // Standard mersenne_twister_engine seeded with rd()
				std::uniform_real_distribution<> dis(0.0, 1.0);
				//int num_of_rays = 50;// trisAreas[t - 1] * 50;
				//cout << num_of_rays << endl;
				for (int n = 0; n < num_of_rays_per_tris; ++n) {
					double r1 = sqrt(dis(gen));
					double r2 = dis(gen);
					double A = (1.0 - r1);
					double B = (r1 * (1.0 - r2));
					double C = (r1 * r2);
					sendRay(triangle, vertices, startPoint, A, B, C);
				}

				/*
				int raysss = 0;
				for (double a = 0; a < 11; a += 0.6) {
					for (double b = 0; b < (11 - a); b += 0.4) {
						double c = 10 - (a + b);

						double A = (double)a / 10.0;
						double B = (double)b / 10.0;
						double C = (double)c / 10.0;
						if (A > 1.0)
							A = 1.0;
						if (B > 1.0)
							B = 1.0;
						if (C > 1.0)
							C = 1.0;
						if (C < 0)
							C = 0;

						raysss++;

						sendRay(triangle, vertices, startPoint, A, B, C);
					}
				}
				cout << "rays: " << raysss << endl;
				*/
				
			}
		}
	}


	for (int v = 0; v < 18; v++)
		delete vertices[v];
	for (int t = 0; t < 6; t++)
		delete triangles[t];

}

void KernelApproximation::expandByRegularTris() {

	double leastCoordinates[3] = { extremeCorners[0][0], extremeCorners[0][1], extremeCorners[0][2] };
	double mostCoordinates[3] = { extremeCorners[1][0], extremeCorners[1][1], extremeCorners[1][2] };

	double coords[8][3] = { {leastCoordinates[0], leastCoordinates[1], leastCoordinates[2]},
							{mostCoordinates[0], leastCoordinates[1], leastCoordinates[2]},
							{mostCoordinates[0], leastCoordinates[1], mostCoordinates[2]},
							{leastCoordinates[0], leastCoordinates[1], mostCoordinates[2]},
							{leastCoordinates[0], mostCoordinates[1], leastCoordinates[2]},
							{mostCoordinates[0], mostCoordinates[1], leastCoordinates[2]},
							{mostCoordinates[0], mostCoordinates[1], mostCoordinates[2]},
							{leastCoordinates[0], mostCoordinates[1], mostCoordinates[2]}
	};

	for (int d = 0; d < 3; d++)
		extremePoints[0][d] = (coords[0][d] + coords[3][d] + coords[4][d] + coords[7][d]) / 4;
	for (int d = 0; d < 3; d++)
		extremePoints[1][d] = (coords[1][d] + coords[2][d] + coords[5][d] + coords[6][d]) / 4;
	for (int d = 0; d < 3; d++)
		extremePoints[2][d] = (coords[0][d] + coords[1][d] + coords[2][d] + coords[3][d]) / 4;
	for (int d = 0; d < 3; d++)
		extremePoints[3][d] = (coords[4][d] + coords[5][d] + coords[6][d] + coords[7][d]) / 4;
	for (int d = 0; d < 3; d++)
		extremePoints[4][d] = (coords[0][d] + coords[1][d] + coords[4][d] + coords[5][d]) / 4;
	for (int d = 0; d < 3; d++)
		extremePoints[5][d] = (coords[2][d] + coords[3][d] + coords[6][d] + coords[7][d]) / 4;

	double AABB_center[3] = { 0, 0, 0 };
	for (int n = 0; n < 6; n++)
		for (int d = 0; d < 3; d++)
			AABB_center[d] += extremePoints[n][d];
	for (int d = 0; d < 3; d++)
		AABB_center[d] /= 6.0;

	// Define triangles in which rays pass through
	vector<Triangle*> triangles;
	vector<Vertex*> vertices;
	vector<double> trisAreas;

	int t = 0;
	for (int i = 0; i < 2; i++) {
		for (int j = 2; j < 4; j++) {
			for (int k = 4; k < 6; k++) {
				int vertex_ids[3] = { t * 3, t * 3 + 1, t * 3 + 2 };
				vertices.push_back(new Vertex(vertex_ids[0], extremePoints[i]));
				vertices.push_back(new Vertex(vertex_ids[1], extremePoints[j]));
				vertices.push_back(new Vertex(vertex_ids[2], extremePoints[k]));

				Triangle* triangle = new Triangle(t, vertex_ids);
				triangles.push_back(triangle);

				double* areaVector = triangle->computeAreaVector(vertices[t * 3], vertices[t * 3 + 1], vertices[t * 3 + 2]);
				trisAreas.push_back(computeLength(areaVector));
				delete[] areaVector;
				t++;
			}
		}
	}

	t = 0;
	for (int i = 0; i < 2; i++) {
		for (int j = 2; j < 4; j++) {
			for (int k = 4; k < 6; k++) {

				Triangle* triangle = triangles[t];
				double* startPoint = AABB_center;	// this->initialPoint;
				t++;

			
				
				std::random_device rd;  // Will be used to obtain a seed for the random number engine
				std::mt19937 gen(19937); // Standard mersenne_twister_engine seeded with rd()
				std::uniform_real_distribution<> dis(0.0, 1.0);
				//int num_of_rays = 50;// trisAreas[t - 1] * 50;
				//cout << num_of_rays << endl;
				for (int n = 0; n < num_of_rays_per_tris; ++n) {
					double r1 = sqrt(dis(gen));
					double r2 = dis(gen);
					double A = (1.0 - r1);
					double B = (r1 * (1.0 - r2));
					double C = (r1 * r2);
					sendRay(triangle, vertices, startPoint, A, B, C);
				}

				/*
				
				for (int a = 1; a < 11; a += 2) {
					for (int b = 1; b < (11 - a); b += 2) {
						int c = 10 - (a + b);

						double A = (double)a / 10.0;
						double B = (double)b / 10.0;
						double C = (double)c / 10.0;
						if (A > 1.0)
							A = 1.0;
						if (B > 1.0)
							B = 1.0;
						if (C > 1.0)
							C = 1.0;
						if (C < 0)
							C = 0;

						sendRay(triangle, vertices, startPoint, A, B, C);
					}
				}

				*/

			}
		}
	}

	for (int v = 0; v < 18; v++)
		delete vertices[v];
	for (int t = 0; t < 6; t++)
		delete triangles[t];

}

void KernelApproximation::expandBySphereSampling() {

	double AABB_center[3] = { 0, 0, 0 };
	for (int n = 0; n < 6; n++)
		for (int d = 0; d < 3; d++)
			AABB_center[d] += extremePoints[n][d];
	for (int d = 0; d < 3; d++)
		AABB_center[d] /= 6.0;

	double startPoint[3] = { AABB_center[0], AABB_center[1], AABB_center[2] };

	int num_of_rays = num_of_rays_per_tris * 8;

	std::mt19937 rnd;
	std::uniform_real_distribution<double> dist(0.0, 1.0);

	for (int i = 0; i < num_of_rays; ++i)
	{
		double z = 2.0 * dist(rnd) - 1.0;
		double t = 2.0 * PI * dist(rnd);
		double r = sqrt(1.0 - z * z);

		double direction[3] = { r * cos(t), r * sin(t), z };
		Line ray(startPoint, direction);

		// intersect with halfspaces
		double closest_distance = numeric_limits<double>::infinity();
		for (int h = 0; h < halfSpaceSet.size(); h++) {
			double tVal = findLinePlaneIntersection(ray, halfSpaceSet[h]);
			if (tVal > 0 && tVal < closest_distance)
				if (dotProduct(halfSpaceSet[h].ABCD, direction) > 0)
					closest_distance = tVal;
		}

		double kernel_point[3];
		for (int d = 0; d < 3; d++)
			kernel_point[d] = startPoint[d] + closest_distance * direction[d];

		kernel.addVertex(kernel_point[0], kernel_point[1], kernel_point[2]);

	}

	/*
	std::mt19937 rnd;
	std::normal_distribution<double> dis(0.0, 1.0);

	for (int i = 0; i < num_of_rays ; ++i)
	{
			bool bad_luck = false;
			do
			{
				double x = dis(rnd);
				double y = dis(rnd);
				double z = dis(rnd);
				double r2 = x * x + y * y + z * z;
				if (r2 == 0)
					bad_luck = true;
				else
				{
					bad_luck = false;
					double r = sqrt(r2);
					double direction[3] = { x / r, y / r, z / r };
					Line ray(startPoint, direction);

					// intersect with halfspaces
					double closest_distance = numeric_limits<double>::infinity();
					for (int h = 0; h < halfSpaceSet.size(); h++) {
						double tVal = findLinePlaneIntersection(ray, halfSpaceSet[h]);
						if (tVal > 0 && tVal < closest_distance)
							if (dotProduct(halfSpaceSet[h].ABCD, direction) > 0)
								closest_distance = tVal;
					}

					double kernel_point[3];
					for (int d = 0; d < 3; d++)
						kernel_point[d] = startPoint[d] + closest_distance * direction[d];

					kernel.addVertex(kernel_point[0], kernel_point[1], kernel_point[2]);

				}
			} while (bad_luck);
		
	}
	*/

}

void KernelApproximation::expandByCornerRays() {

	double leastCoordinates[3] = { extremeCorners[0][0], extremeCorners[0][1], extremeCorners[0][2] };
	double mostCoordinates[3] = { extremeCorners[1][0], extremeCorners[1][1], extremeCorners[1][2] };

	double coords[8][3] = { {leastCoordinates[0], leastCoordinates[1], leastCoordinates[2]},
						{mostCoordinates[0], leastCoordinates[1], leastCoordinates[2]},
						{mostCoordinates[0], leastCoordinates[1], mostCoordinates[2]},
						{leastCoordinates[0], leastCoordinates[1], mostCoordinates[2]},
						{leastCoordinates[0], mostCoordinates[1], leastCoordinates[2]},
						{mostCoordinates[0], mostCoordinates[1], leastCoordinates[2]},
						{mostCoordinates[0], mostCoordinates[1], mostCoordinates[2]},
						{leastCoordinates[0], mostCoordinates[1], mostCoordinates[2]}
	};

	// send rays to grid corners
	for (int i = 0; i < 8; i++) {
		double direction[3];
		for (int d = 0; d < 3; d++)
			direction[d] = coords[i][d] - this->initialPoint[d];
		normalize(direction);
		Line ray(this->initialPoint, direction);

		// intersect with halfspaces
		double closest_distance = numeric_limits<double>::infinity();
		for (int h = 0; h < halfSpaceSet.size(); h++) {
			double tVal = findLinePlaneIntersection(ray, halfSpaceSet[h]);
			if (tVal > EPSILON && tVal < closest_distance)
				if (dotProduct(halfSpaceSet[h].ABCD, direction) > 0)
					closest_distance = tVal;
		}

		double kernel_point[3];
		for (int d = 0; d < 3; d++)
			kernel_point[d] = this->initialPoint[d] + closest_distance * direction[d];

		kernel.addVertex(kernel_point[0], kernel_point[1], kernel_point[2]);
	}

}

void KernelApproximation::sendRay(Triangle* triangle, vector<Vertex*>& vertices, double* startPoint, double a, double b, double c) {

	// compute coordinates of the point
	double point[3];
	for (int d = 0; d < 3; d++)
		point[d] = a * vertices[triangle->corners[0]]->coords[d] +
		b * vertices[triangle->corners[1]]->coords[d] +
		c * vertices[triangle->corners[2]]->coords[d];

	// define ray
	double direction[3];
	for (int d = 0; d < 3; d++)
		direction[d] = point[d] - startPoint[d];
	normalize(direction);
	Line ray(startPoint, direction);

	// intersect with halfspaces
	double closest_distance = numeric_limits<double>::infinity();
	for (int h = 0; h < halfSpaceSet.size(); h++) {
		double tVal = findLinePlaneIntersection(ray, halfSpaceSet[h]);
		if (tVal > 0 && tVal < closest_distance)
			if (dotProduct(halfSpaceSet[h].ABCD, direction) > 0)
				closest_distance = tVal;
	}

	double kernel_point[3];
	for (int d = 0; d < 3; d++)
		kernel_point[d] = startPoint[d] + closest_distance * direction[d];

	kernel.addVertex(kernel_point[0], kernel_point[1], kernel_point[2]);

}

void KernelApproximation::expandByExactVerts() {
	
	double AABB_center[3] = { 0, 0, 0 };
	for (int n = 0; n < 6; n++)
		for (int d = 0; d < 3; d++)
			AABB_center[d] += extremePoints[n][d];
	for (int d = 0; d < 3; d++)
		AABB_center[d] /= 6.0;

	vector<int> plane_ids;
	for (int i = 0; i < kernel.getNumOfVerts(); i++)
		plane_ids.push_back(findOwnerPlane(kernel.getVertex(i).coords));

	
	int num_of_verts = kernel.getNumOfVerts();
	for (int i = 0; i < num_of_verts; i++) {

		Vertex v = kernel.getVertex(i);
		int plane1_id = findOwnerPlane(v.coords);

		for (int j = 0; j < v.vertList.size(); j++) {
			
			Vertex vn1 = kernel.getVertex(v.vertList[j]);
			if (v.idx > vn1.idx)
				continue;
			int plane2_id = findOwnerPlane(vn1.coords);

			for (int k = j+1; k < v.vertList.size(); k++) {

				Vertex vn2 = kernel.getVertex(v.vertList[k]);
				if (v.idx > vn2.idx)
					continue;
				int plane3_id = findOwnerPlane(vn2.coords);

				possibleVertexProducerIds.push(new int[4]{ plane1_id, plane2_id, plane3_id, 0 });
				findClosestKernelVertex(AABB_center);
			}

		}

	}
	

}

int KernelApproximation::findOwnerPlane(double* coords) {

	for (int h = 0; h < halfSpaceSet.size(); h++) {
		double value = halfSpaceSet[h].ABCD[3];
		for (int d = 0; d < 3; d++)
			value += halfSpaceSet[h].ABCD[d] * coords[d];
		if (abs(value) < EPSILON) {
			return h;
		}
	}

	return -1;

}

void KernelApproximation::findClosestKernelVertex(double source_coords[3]) {

	for (int i = 0; !possibleVertexProducerIds.empty(); i++) {
		// take the intersection point of the found planes
		int* closestPlaneIds = possibleVertexProducerIds.front();
		double* candidateKernelVertex = NULL;
		if (closestPlaneIds[0] >= 0 && closestPlaneIds[1] >= 0 && closestPlaneIds[2] >= 0) {
			Plane closestPlanes[3] = { halfSpaceSet[closestPlaneIds[0]], halfSpaceSet[closestPlaneIds[1]], halfSpaceSet[closestPlaneIds[2]] };
			candidateKernelVertex = find3PlaneIntersection(halfSpaceSet[closestPlaneIds[0]], halfSpaceSet[closestPlaneIds[1]], halfSpaceSet[closestPlaneIds[2]]);
		}

		// stop by the found vertex or keep to search
		if (candidateKernelVertex) {
			if (isAlreadyKernelVertex(candidateKernelVertex))	// is this vertex found before?
				;												// then do not go into loop one more time
			else {
				double* candidateScalarsVector = findValueVectorSatisfiedByPoint(candidateKernelVertex, halfSpaceSet);
				if (isInKernel(candidateScalarsVector))
					kernel.addVertex(candidateKernelVertex[0], candidateKernelVertex[1], candidateKernelVertex[2]);
				else if (closestPlaneIds[3] > recursion_depth) {
					int obstaclePlaneId = findIfFrontierObstacle(candidateKernelVertex, source_coords, closestPlaneIds, candidateScalarsVector);
					possibleVertexProducerIds.push(new int[4]{ closestPlaneIds[0], closestPlaneIds[1], obstaclePlaneId, closestPlaneIds[3] - 1 });
					possibleVertexProducerIds.push(new int[4]{ closestPlaneIds[0], closestPlaneIds[2], obstaclePlaneId, closestPlaneIds[3] - 1 });
					possibleVertexProducerIds.push(new int[4]{ closestPlaneIds[1], closestPlaneIds[2], obstaclePlaneId, closestPlaneIds[3] - 1 });
				}
				delete[] candidateScalarsVector;
			}
			delete[] candidateKernelVertex;
		}

		delete[] closestPlaneIds;
		possibleVertexProducerIds.pop();
		if (possibleVertexProducerIds.empty())
			return;
	}
}

bool KernelApproximation::isAlreadyKernelVertex(double coords[3]) {

	for (int i = 0; i < kernel.getNumOfVerts(); i++) {
		Vertex v = kernel.getVertex(i);
		if (abs(v.coords[0] - coords[0]) < EPSILON && abs(v.coords[1] - coords[1]) < EPSILON && abs(v.coords[2] - coords[2]) < EPSILON)
			return true;
	}
	return false;
}

int KernelApproximation::findIfFrontierObstacle(double* candidateKernelVertex, double* sourcePoint, int candidateVertexProducerIds[3], double* scalarVector) {

	int frontierHalfSpaceId = -1;

	double direction[3];
	for (int i = 0; i < 3; i++)
		direction[i] = candidateKernelVertex[i] - sourcePoint[i];
	double candidateDistance = computeLength(direction);

	normalize(direction);
	Line line(sourcePoint, direction);

	for (int i = 0; i < halfSpaceSet.size(); i++) {
		if (scalarVector[i] > 0) {
			double distance = findLinePlaneIntersection(line, halfSpaceSet[i]);
			if (distance > 0 && distance < candidateDistance) {
				frontierHalfSpaceId = i;
				candidateDistance = distance;
			}
		}
	}

	return frontierHalfSpaceId;

}
