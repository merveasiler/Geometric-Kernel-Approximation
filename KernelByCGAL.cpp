// @taken_from https://doc.cgal.org/latest/Convex_hull_3/index.html

#include <CGAL/Convex_hull_3/dual/halfspace_intersection_with_constructions_3.h>
#include <CGAL/Convex_hull_3/dual/halfspace_intersection_3.h>
#include <list>

#include "KernelByCGAL.h"
#include "BaseGeoOpUtils.h"
#include "CGALUtils.h"

KernelByCGAL::KernelByCGAL(const Mesh& hostMesh) :
    KernelExpansion(hostMesh) {

    // Construct the halfspace inequality system which defines the kernel using input mesh triangles
    this->halfSpaceCoeffs = computeHalfSpaceCoeffsFromTriangles(hostMesh.getAllTris(), hostMesh.getAllVerts());

};

KernelByCGAL::~KernelByCGAL() {

}

/*
* @function expandKernel
* @abstract Computes the kernel of an input mesh using CGAL's algorithm (Preparata et al., 1979)
*/
void KernelByCGAL::expandKernel() {

    // extract the A, B, C, D coefficients of a plane equation of the form Ax + By + Cz + D = 0
    // construct those planes as CGAL planes 
    std::list<CGALPlane> planes;
    for (int i = 0; i < halfSpaceCoeffs.size(); i++) {
        double* coeffs = halfSpaceCoeffs[i];
        typename K::Plane_3 plane(coeffs[0], coeffs[1], coeffs[2], coeffs[3]);
        planes.push_back(plane);
        delete[] coeffs;
    }
    halfSpaceCoeffs.clear();

    // Kernel is equal to the intersection of halfspaces bounding the input mesh 
    CGALMesh chull;
    CGAL::halfspace_intersection_with_constructions_3(planes.begin(), planes.end(), chull);
    //CGAL::halfspace_intersection_3(planes.begin(), planes.end(), chull);    // if no point inside the intersection is provided, one will be automatically found using linear programming
    //CGAL::halfspace_intersection_3(planes.begin(), planes.end(), chull, CGALPoint(this->extremePoints[0][0], this->extremePoints[0][1], this->extremePoints[0][2]));   
    //std::cout << "The convex hull contains " << num_vertices(chull) << " vertices" << std::endl;

   
    // eliminate nan(ind) valued points
    Mesh tempMesh;
    for (CGALMesh::Vertex_index vi : chull.vertices()) {
        CGALPoint pt = chull.point(vi);
        if (isnan((double)pt.x()) || isnan((double)pt.y()) || isnan((double)pt.z()))
            continue;
        tempMesh.addVertex((double)pt.x(), (double)pt.y(), (double)pt.z());
    }
    this->kernel = computeConvexHull(tempMesh.getAllVerts());   
    
    //this->kernel = convertCGALMeshToMesh(chull);
}

