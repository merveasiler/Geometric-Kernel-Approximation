// @author Merve Asiler

#include "KernelApproximation.h"
#include "BaseGeoOpUtils.h"
#include "CGALUtils.h"
#include <random>

/***********************************/
/***** Defined in Parameters.h *****/
extern int number_of_rays;
extern int recursion_depth_limit;
extern string ray_distribution_type;
/***********************************/

/*
*	@function	KernelApproximation
*	@abstract	Constructor : defines the AABB (axis-aligned bounding box) of the kernel using the extreme kernel points in the six principal directions (+x, -x, +y, -y, +z, -z).
*/
KernelApproximation::KernelApproximation(const Mesh& hostMesh) :
	
	// Find the initial kernel points on the extreme directions
	//    and 
	// Define the AABB 
	KernelExpansion(hostMesh) {

};

/*
*	@function	~KernelApproximation
*	@abstract	Destructor
*/
KernelApproximation::~KernelApproximation() {


}

/*
*	@function	expandKernel
*	@abstract	manages the whole kernel computation process by calling the corresponding function for each phase.
*/
void KernelApproximation::expandKernel() {

	// Is kernel empty?
	if (this->is_kernel_empty)
		return;	// NOT STAR-SHAPED!

	// Distribute rays according to distribution type
	if (ray_distribution_type == "geometry-based")
		expandByExtremeTris();
	else
		expandBySphereSampling();

	// Send rays to the corners of the AABB. Optionally you can take it into comments.
	expandByCornerRays();	

	// Delineate the initial kernel boundary
	kernel = computeConvexHull(kernel.getAllVerts());

	// Recursively search for the kernel vertices
	expandByExactVerts();

	// In case of a precision error, apply a basic filter to check whether the vertices really belong to the kernel
	vector <Vertex> filteredVertices;
	for (int v = 0; v < kernel.getNumOfVerts(); v++) {
		double* scalars = findValueVectorSatisfiedByPoint(kernel.getVertex(v).coords, halfSpaceSet);
		if (isInKernel(scalars))
			filteredVertices.push_back(kernel.getVertex(v));
		delete[] scalars;
	}

	// Extract the final kernel boundary 
	kernel = computeConvexHull(filteredVertices);

}

/*
*	@function	expandByExtremeTris
*	@abstract	Uniformly samples the triangles structured around the corners of the AABB.
*				Sends a ray in the direction from the AABB center to the sample point.
*				Finds the boundary kernel point in that direction.
*/
void KernelApproximation::expandByExtremeTris() {

	// Define triangles (around the corners of the AABB) in which rays pass through
	vector<Triangle*> triangles;
	vector<Vertex*> vertices;
	
	// Construct each corner triangle using the closest extreme kernel points around the corner 
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
				t++;
			}
		}
	}

	// equally distribute rays to each triangle
	int num_of_rays_per_tris = number_of_rays / 8;	// one triangle per each of the eight corners of the AABB

	// for each triangle:
	t = 0;
	for (int i = 0; i < 2; i++) {
		for (int j = 2; j < 4; j++) {
			for (int k = 4; k < 6; k++) {

				Triangle* triangle = triangles[t];
				double* startPoint = AABB_center;	// initialize the source point of the rays as the center of the AABB
				t++;

				// uniformly sample the triangle
				std::random_device rd;		// Will be used to obtain a seed for the random number engine
				std::mt19937 gen(19937);	// Standard mersenne_twister_engine seeded with rd()
				std::uniform_real_distribution<> dis(0.0, 1.0);

				for (int n = 0; n < num_of_rays_per_tris; ++n) {
					double r1 = sqrt(dis(gen));
					double r2 = dis(gen);
					// A, B, C are the barycentric coordinates
					double A = (1.0 - r1);
					double B = (r1 * (1.0 - r2));
					double C = (r1 * r2);
					// define the ray to pass through the point with the barycentric coordinate <A,B,C>
					sendRay(triangle, vertices, startPoint, A, B, C);
				}

				
			}
		}
	}

	// clear up
	for (int v = 0; v < 18; v++)
		delete vertices[v];
	for (int t = 0; t < 6; t++)
		delete triangles[t];

}

/*
*	@function	expandBySphereSampling
*	@abstract	Uniformly samples the unit sphere centered at the center of the AABB.
*				Sends a ray in the direction from the AABB center to the sample point.
*				Finds the boundary kernel point in that direction. 
*/
void KernelApproximation::expandBySphereSampling() {

	// initialize the source point of the rays as the center of the AABB
	double* startPoint = AABB_center;

	// apply uniform spherical distribution
	std::mt19937 rnd;
	std::uniform_real_distribution<double> dist(0.0, 1.0);

	for (int i = 0; i < number_of_rays; ++i)
	{
		double z = 2.0 * dist(rnd) - 1.0;
		double t = 2.0 * PI * dist(rnd);
		double r = sqrt(1.0 - z * z);

		double direction[3] = { r * cos(t), r * sin(t), z };	// ray direction
		Line ray(startPoint, direction);						// ray centered at startPoint

		// intersect with halfspaces, pick the closest intersection to startPoint
		double closest_distance = numeric_limits<double>::infinity();
		for (int h = 0; h < halfSpaceSet.size(); h++) {
			double tVal = findLinePlaneIntersection(ray, halfSpaceSet[h]);
			if (tVal > 0 && tVal < closest_distance)
				if (dotProduct(halfSpaceSet[h].ABCD, direction) > 0)
					closest_distance = tVal;
		}

		// finalize the closest hit using the closest intersection distance
		double kernel_point[3];
		for (int d = 0; d < 3; d++)
			kernel_point[d] = startPoint[d] + closest_distance * direction[d];

		// this point is inside the kernel, save it as a boundary kernel point
		kernel.addVertex(kernel_point[0], kernel_point[1], kernel_point[2]);

	}
 
	/*
	* ANOTHER COMMON TECHNIQUE FOR UNIFORM SPHERE SAMPLING
	* 
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

/*
*	@function	expandByCornerRays
*	@abstract	Sends one ray per corner of the AABB to find the boundary kernel point in that direction.
*/
void KernelApproximation::expandByCornerRays() {

	// the cross corners of the AABB in the directions (-x, -y, -z) and (x, y, z)
	double leastCoordinates[3] = { extremeCorners[0][0], extremeCorners[0][1], extremeCorners[0][2] };
	double mostCoordinates[3] = { extremeCorners[1][0], extremeCorners[1][1], extremeCorners[1][2] };

	// extract the coordinates of all the corners of the AABB
	double coords[8][3] = { {leastCoordinates[0], leastCoordinates[1], leastCoordinates[2]},
						{mostCoordinates[0], leastCoordinates[1], leastCoordinates[2]},
						{mostCoordinates[0], leastCoordinates[1], mostCoordinates[2]},
						{leastCoordinates[0], leastCoordinates[1], mostCoordinates[2]},
						{leastCoordinates[0], mostCoordinates[1], leastCoordinates[2]},
						{mostCoordinates[0], mostCoordinates[1], leastCoordinates[2]},
						{mostCoordinates[0], mostCoordinates[1], mostCoordinates[2]},
						{leastCoordinates[0], mostCoordinates[1], mostCoordinates[2]}
	};

	// send rays to the AABB corners
	for (int i = 0; i < 8; i++) {
		double direction[3];
		for (int d = 0; d < 3; d++)
			direction[d] = coords[i][d] - this->AABB_center[d];	// ray direction
		normalize(direction);
		Line ray(this->AABB_center, direction);					// define the ray passing through the center of the AABB

		// intersect with halfspaces, pick the closest intersection to initialPoint
		double closest_distance = numeric_limits<double>::infinity();
		for (int h = 0; h < halfSpaceSet.size(); h++) {
			double tVal = findLinePlaneIntersection(ray, halfSpaceSet[h]);
			if (tVal > EPSILON && tVal < closest_distance)
				if (dotProduct(halfSpaceSet[h].ABCD, direction) > 0)
					closest_distance = tVal;
		}

		// finalize the closest hit using the closest intersection distance
		double kernel_point[3];
		for (int d = 0; d < 3; d++)
			kernel_point[d] = this->AABB_center[d] + closest_distance * direction[d];

		// this point is inside the kernel, add it as a boundary kernel point
		kernel.addVertex(kernel_point[0], kernel_point[1], kernel_point[2]);
	}

}

/*
*	@function					sendRay
*	@abstract					Sends a ray with the given source and target points (encoded). 
								It records the intersection point if there is an early hit to some other plane).
*	@param		triangle		the triangle to which the ray is sent
*	@param		a, b, c			the barycentric coordiantes of the target point on the <triangle>
*	@param		startPoint		the start point of the ray.
*	@result						It computes the hit of the ray which is closest to the <startPoint>. It records this point as a kernel point.
*/
void KernelApproximation::sendRay(Triangle* triangle, vector<Vertex*>& vertices, double* startPoint, double a, double b, double c) {

	// compute the Eucledean coordinates of the point using the barycentric coordinates
	double point[3];
	for (int d = 0; d < 3; d++)
		point[d] = a * vertices[triangle->corners[0]]->coords[d] +
		b * vertices[triangle->corners[1]]->coords[d] +
		c * vertices[triangle->corners[2]]->coords[d];

	// define the ray sourcing from startPoint and directioned towards the target triangle point 
	double direction[3];
	for (int d = 0; d < 3; d++)
		direction[d] = point[d] - startPoint[d];
	normalize(direction);
	Line ray(startPoint, direction);

	// intersect with halfspaces, pick the closest intersection to startPoint
	double closest_distance = numeric_limits<double>::infinity();
	for (int h = 0; h < halfSpaceSet.size(); h++) {
		double tVal = findLinePlaneIntersection(ray, halfSpaceSet[h]);
		if (tVal > 0 && tVal < closest_distance)
			if (dotProduct(halfSpaceSet[h].ABCD, direction) > 0)
				closest_distance = tVal;
	}

	// finalize the closest hit using the closest intersection distance
	double kernel_point[3];
	for (int d = 0; d < 3; d++)
		kernel_point[d] = startPoint[d] + closest_distance * direction[d];

	// this point is inside the kernel, save it as a boundary kernel point
	kernel.addVertex(kernel_point[0], kernel_point[1], kernel_point[2]);

}

/*
*	@function	expandByExactVerts
*	@abstract	find a nearby kernel vertex for each previously found kernel points (previously found by ray tracing).
*/
void KernelApproximation::expandByExactVerts() {

	// for each of vertex of the current partial kernel: 
	//     ... first find the plane on which the point lies, record its id into plane1_id
	int num_of_verts = kernel.getNumOfVerts();
	for (int i = 0; i < num_of_verts; i++) {

		Vertex v = kernel.getVertex(i);
		int plane1_id = findOwnerPlane(v.coords);

		// for each adjacent vertex of the previous vertex
		//		... find the second plane as the plane on which its adjacent vertex lies, record its id into plane2_id
		for (int j = 0; j < v.vertList.size(); j++) {
			
			Vertex vn1 = kernel.getVertex(v.vertList[j]);		// from the adjacency vertex list
			if (v.idx > vn1.idx)
				continue;
			int plane2_id = findOwnerPlane(vn1.coords);

			// for each remaining adjacent vertex of the first vertex
			//		... find the third plane as the plane on which its adjacent vertex lies, record its id into plane3_id
			for (int k = j+1; k < v.vertList.size(); k++) {

				Vertex vn2 = kernel.getVertex(v.vertList[k]);	// from the adjacency vertex list
				if (v.idx > vn2.idx)
					continue;
				int plane3_id = findOwnerPlane(vn2.coords);

				// for the determined planes, initiate the recursive search process
				possibleVertexProducerIds.push(new int[4]{ plane1_id, plane2_id, plane3_id, 0 });	// the last index is the depth of the recursive process
				findClosestKernelVertex();
			}

		}

	}
	
}

/*
*	@function	findOwnerPlane
*	@abstract	find the closest bounding plane to the point defined by the coordinates <coords>. The corresponding half-space must be containing the point.
*/
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

/*
*	@function	findClosestKernelVertex	
*	@abstract	the recursive search process to find the closest kernel vertex to a reference point/vertex.
				In this implementation, the id(s) of the plane(s) passing through the reference vertex and its adjacent points are held
				in a class variable named as possibleVertexProducerIds.
*/
void KernelApproximation::findClosestKernelVertex() {

	// pop a triplet of planes which is candidate to generate a kernel vertex:
	// find the intersection of the planes in the triplet
	for (int i = 0; !possibleVertexProducerIds.empty(); i++) {
		int* closestPlaneIds = possibleVertexProducerIds.front();
		double* candidateKernelVertex = NULL;
		if (closestPlaneIds[0] >= 0 && closestPlaneIds[1] >= 0 && closestPlaneIds[2] >= 0) {
			Plane closestPlanes[3] = { halfSpaceSet[closestPlaneIds[0]], halfSpaceSet[closestPlaneIds[1]], halfSpaceSet[closestPlaneIds[2]] };
			candidateKernelVertex = find3PlaneIntersection(halfSpaceSet[closestPlaneIds[0]], halfSpaceSet[closestPlaneIds[1]], halfSpaceSet[closestPlaneIds[2]]);
		}

		// stop by the found vertex or keep on searching:

		// if the given plane triple indeed intersect at a vertex
		if (candidateKernelVertex) {	// then this variable is not null
			if (isAlreadyKernelVertex(candidateKernelVertex))	// is this vertex found before?
				;												// then do not go into loop one more time
			else {
				// save the signed distances of the found vertex to each bounding plane
				double* candidateScalarsVector = findValueVectorSatisfiedByPoint(candidateKernelVertex, halfSpaceSet);
				if (isInKernel(candidateScalarsVector))	// if sign of each distance is negative, then this is a kernel vertex 
					kernel.addVertex(candidateKernelVertex[0], candidateKernelVertex[1], candidateKernelVertex[2]);	// then save this kernel vertex
				// if the vertex is not a kernel vertex, then there is an obstacle plane between AABB_center and the vertex
				// ... find that obstacle plane and keep recursive search by replacing one of the previous candidates with that obstacle plane
				else if (closestPlaneIds[3] < recursion_depth_limit) {	// if we are still below the recursive search depth limit
					// find the closest plane to AABB_center on the path to candidateKernelVertex
					int obstaclePlaneId = findIfFrontierObstacle(candidateKernelVertex, AABB_center, candidateScalarsVector);
					// pply replacements of the obstacle plane
					possibleVertexProducerIds.push(new int[4]{ closestPlaneIds[0], closestPlaneIds[1], obstaclePlaneId, closestPlaneIds[3] + 1 });
					possibleVertexProducerIds.push(new int[4]{ closestPlaneIds[0], closestPlaneIds[2], obstaclePlaneId, closestPlaneIds[3] + 1 });
					possibleVertexProducerIds.push(new int[4]{ closestPlaneIds[1], closestPlaneIds[2], obstaclePlaneId, closestPlaneIds[3] + 1 });
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

/*
*	@function	isAlreadyKernelVertex
*	@abstract	check if the given point with coordinates <coords> is a previously found vertex of the kernel.
*/
bool KernelApproximation::isAlreadyKernelVertex(double coords[3]) {

	for (int i = 0; i < kernel.getNumOfVerts(); i++) {
		Vertex v = kernel.getVertex(i);
		if (abs(v.coords[0] - coords[0]) < EPSILON && abs(v.coords[1] - coords[1]) < EPSILON && abs(v.coords[2] - coords[2]) < EPSILON)
			return true;
	}
	return false;
}

/*
*	@function								findIfFrontierObstacle
*	@abstract								check if there is a another plane (an obstacle plane) on the path between the source and the target points.
*	@param		sourcePoint					the source point coordinates				
*	@param		candidateKernelVertex		the target point coordinates
*	@param		scalarVector				the signed distances of the candidateKernelVertex to each bounding plane
*	@result									the id of the obstacle plane. If there is no obstacle, return -1.				
*/
int KernelApproximation::findIfFrontierObstacle(double* candidateKernelVertex, double* sourcePoint, double* scalarVector) {

	int frontierHalfSpaceId = -1;

	// direction vector from the source point to the target point
	double direction[3];
	for (int i = 0; i < 3; i++)
		direction[i] = candidateKernelVertex[i] - sourcePoint[i];

	// the distance between the source point to the target point
	double candidateDistance = computeLength(direction);

	// define the ray starting at sourcePoint and going towards the target point
	normalize(direction);
	Line ray(sourcePoint, direction);


	// for each bounding plane check if they hit the ray before reaching at the target point
	for (int i = 0; i < halfSpaceSet.size(); i++) {
		if (scalarVector[i] > 0) {	// if this bounding plane does not contain the target point (=candidateKernelVertex)
			double distance = findLinePlaneIntersection(ray, halfSpaceSet[i]);	// check if it istersects the ray
			if (distance > 0 && distance < candidateDistance) {	// If it intersects the ray (line) between the source and the target point
				frontierHalfSpaceId = i;						// ... then it is an obstacle plane.
				candidateDistance = distance;					// ... we record only the obstacle plane which is closest to the source
			}
		}
	}

	// return the obstacle (frontier plane) id
	// ... if there is no obstacle, this variable remains as -1
	return frontierHalfSpaceId;

}
