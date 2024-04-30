// @author Merve Asiler

#include "CGALUtils.h"

Mesh convertCGALMeshToMesh(CGALMesh sm) {

    Mesh mesh;

    for (CGALMesh::Vertex_index vi : sm.vertices()) {
        CGALPoint pt = sm.point(vi);
        mesh.addVertex((double)pt.x(), (double)pt.y(), (double)pt.z());
    }

    for (CGALMesh::Face_index face_index : sm.faces()) {
        vector<uint32_t> indices;
        for (CGALMesh::Vertex_index vi : vertices_around_face(sm.halfedge(face_index), sm))
            indices.push_back(vi.idx());
        while (indices.size() >= 3) {
            mesh.addTriangle(indices[0], indices[1], indices[2]);
            indices.erase(indices.begin() + 1);
        }
    }

    return mesh;
}

Mesh convertCGALMeshToMesh(CGALMesh sm, vector<vector<int>>& vertex_groups) {

    Mesh mesh;

    for (CGALMesh::Vertex_index vi : sm.vertices()) {
        CGALPoint pt = sm.point(vi);
        mesh.addVertex((double)pt.x(), (double)pt.y(), (double)pt.z());
    }

    for (CGALMesh::Face_index face_index : sm.faces()) {
        vector<uint32_t> indices;
        for (CGALMesh::Vertex_index vi : vertices_around_face(sm.halfedge(face_index), sm))
            indices.push_back(vi.idx());
        
        vector<int> vertex_group;
        for (int v = 0; v < indices.size(); v++)
            vertex_group.push_back(indices[v]);
        vertex_groups.push_back(vertex_group);

        while (indices.size() >= 3) {
            mesh.addTriangle(indices[0], indices[1], indices[2]);
            indices.erase(indices.begin() + 1);
        }
    }

    return mesh;
}

Mesh convertCGALPolyhedronToMesh(CGALPolyhedron poly) {

    Mesh mesh;

    // save vertices
    for (Vertex_iterator i = poly.points_begin(); i != poly.points_end(); ++i)
        mesh.addVertex(i->x(), i->y(), i->z());

    // save triangles
    for (Facet_iterator i = poly.facets_begin(); i != poly.facets_end(); ++i) {
        Halfedge_facet_circulator j = i->facet_begin();
        vector<int> vertex_indices;
        do {
            vertex_indices.push_back(std::distance(poly.vertices_begin(), j->vertex()));
        } while (++j != i->facet_begin());
        while (vertex_indices.size() >= 3) {
            mesh.addTriangle(vertex_indices[0], vertex_indices[1], vertex_indices[2]);
            vertex_indices.erase(vertex_indices.begin() + 1);
        }
    }

    return mesh;

}

CGALMesh convertMeshToCGALMesh(Mesh& mesh) {

    CGALMesh surface_mesh;
    vector<CGALMesh::Vertex_index> vi;

    // Add the points as vertices
    for (int i = 0; i < mesh.getNumOfVerts(); i++) {
        Vertex v = mesh.getVertex(i);
        vi.push_back(surface_mesh.add_vertex(CGALPoint(v.coords[0], v.coords[1], v.coords[2])));
    }

    // Add the triangles as faces
    for (int i = 0; i < mesh.getNumOfTris(); i++) {
        Triangle t = mesh.getTriangle(i);
        CGALMesh::Face_index fi = surface_mesh.add_face(vi[t.corners[0]], vi[t.corners[1]], vi[t.corners[2]]);
    }

    vi.clear();
    return surface_mesh;

}

CGALMesh convertMeshToCGALGraphMesh(Mesh& mesh) {

    CGALMesh surface_mesh;
    vector<CGALPoint> points;

    // Add the points as vertices
    for (int i = 0; i < mesh.getNumOfVerts(); i++) {
        Vertex v = mesh.getVertex(i);
        points.push_back(CGALPoint(v.coords[0], v.coords[1], v.coords[2]));
    }

    // Add the triangles as faces
    for (int i = 0; i < mesh.getNumOfTris(); i++) {
        Triangle t = mesh.getTriangle(i);
        CGAL::make_triangle(points[t.corners[0]], points[t.corners[1]], points[t.corners[2]], surface_mesh);
    }
    
    points.clear();
    return surface_mesh;

}

double* computeHausdorffDistance(Mesh& mesh1, Mesh& mesh2) {

    CGALMesh sm1 = convertMeshToCGALMesh(mesh1);
    CGALMesh sm2 = convertMeshToCGALMesh(mesh2);

    double* hd = new double[3];
    hd[0] = CGAL::Polygon_mesh_processing::approximate_Hausdorff_distance<TAG>(sm1, sm2);
    hd[1] = CGAL::Polygon_mesh_processing::approximate_Hausdorff_distance<TAG>(sm2, sm1);
    hd[2] = CGAL::Polygon_mesh_processing::approximate_symmetric_Hausdorff_distance <TAG> (sm1, sm2);
    return hd;

}

Mesh computeConvexHull(const vector<Vertex>& vertices) {

    // convert vertices to CGALPoint array
    std::vector<CGALPoint> points;
    for (int i = 0; i < vertices.size(); i++) {
        Vertex v = vertices[i];
        CGALPoint p(v.coords[0], v.coords[1], v.coords[2]);
        points.push_back(p);
    }
    
    // define polyhedron to hold convex hull
    CGALMesh chull;
    // compute convex hull of non-collinear points
    CGAL::convex_hull_3(points.begin(), points.end(), chull);
    return convertCGALMeshToMesh(chull);
}

Mesh computeConvexHull(const vector<Vertex>& vertices, vector<vector<int>>& vertex_groups) {

    // convert vertices to CGALPoint array
    std::vector<CGALPoint> points;
    for (int i = 0; i < vertices.size(); i++) {
        Vertex v = vertices[i];
        CGALPoint p(v.coords[0], v.coords[1], v.coords[2]);
        points.push_back(p);
    }

    // define polyhedron to hold convex hull
    CGALMesh chull;
    // compute convex hull of non-collinear points
    CGAL::convex_hull_3(points.begin(), points.end(), chull);
    return convertCGALMeshToMesh(chull, vertex_groups);
}

Mesh computeConvexHull(string meshName) {
    Mesh inputMesh;
    if (meshName.substr(meshName.length() - 3, 3) == "off")
        inputMesh.loadOff(meshName.c_str());
    else
        inputMesh.loadObj(meshName.c_str());

    Mesh mesh = computeConvexHull(inputMesh.getAllVerts());
    return mesh;

}
