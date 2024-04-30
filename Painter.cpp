// @author Merve Asiler

#include "Painter.h"
#include "BaseGeoOpUtils.h"

void Painter::getShapeSep(Mesh* mesh, SoSeparator* res) {

	// Paint all vertices with the same color
	SoMaterial* mat = new SoMaterial();
	mat->diffuseColor.set1Value(0, 1, 1, 1);
	mat->transparency = 0.5;
	res->addChild(mat);

	// Gouraud shading
	SoShapeHints* hints = new SoShapeHints;
	hints->creaseAngle = 3.14;
	res->addChild(hints);

	SoCoordinate3* coords = new SoCoordinate3();
	for (int c = 0; c < mesh->getNumOfVerts(); c++)
		coords->point.set1Value(c, mesh->getVertex(c).coords[0],
			mesh->getVertex(c).coords[1],
			mesh->getVertex(c).coords[2]);

	SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
	for (int c = 0; c < mesh->getNumOfTris(); c++)
	{
		faceSet->coordIndex.set1Value(c * 4, mesh->getTriangle(c).corners[0]);
		faceSet->coordIndex.set1Value(c * 4 + 1, mesh->getTriangle(c).corners[1]);
		faceSet->coordIndex.set1Value(c * 4 + 2, mesh->getTriangle(c).corners[2]);
		faceSet->coordIndex.set1Value(c * 4 + 3, -1);
	}

	res->addChild(coords);
	res->addChild(faceSet);
}

void Painter::getShapeSep(Mesh* mesh, MaterialSetting* materialSetting, SoSeparator* res) {

	// Paint all vertices with the same color
	SoMaterial* mat = new SoMaterial();	
	mat->transparency = materialSetting->getTransparency();					
	res->addChild(mat);

	// Gouraud shading
	SoShapeHints* hints = new SoShapeHints;
	hints->creaseAngle = 3.14;
	res->addChild(hints);

	SoCoordinate3* coords = new SoCoordinate3();
	for (int c = 0; c < mesh->getNumOfVerts(); c++) {
		coords->point.set1Value(c, mesh->getVertex(c).coords[0],
			mesh->getVertex(c).coords[1],
			mesh->getVertex(c).coords[2]);
		mat->diffuseColor.set1Value(c, materialSetting->getRed(), materialSetting->getGreen(), materialSetting->getBlue());
	}

	SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
	for (int c = 0; c < mesh->getNumOfTris(); c++)
	{
		faceSet->coordIndex.set1Value(c * 4, mesh->getTriangle(c).corners[0]);
		faceSet->coordIndex.set1Value(c * 4 + 1, mesh->getTriangle(c).corners[1]);
		faceSet->coordIndex.set1Value(c * 4 + 2, mesh->getTriangle(c).corners[2]);
		faceSet->coordIndex.set1Value(c * 4 + 3, -1);		

		faceSet->materialIndex.set1Value(0 + 4 * c, mesh->getTriangle(c).corners[0]);
		faceSet->materialIndex.set1Value(1 + 4 * c, mesh->getTriangle(c).corners[1]);
		faceSet->materialIndex.set1Value(2 + 4 * c, mesh->getTriangle(c).corners[2]);
	}

	res->addChild(coords);
	res->addChild(faceSet);
}

void Painter::getColorfulShapeSep(Mesh* mesh, SoSeparator* res)
{
	// Gouraud shading
	SoShapeHints* hints = new SoShapeHints;
	hints->creaseAngle = 3.14;
	res->addChild(hints);	//Gouraud shading

	SoMaterial* mat = new SoMaterial();
	mat->transparency = 0;
	res->addChild(mat);

	SoCoordinate3* coords = new SoCoordinate3();
	for (int c = 0; c < mesh->getNumOfVerts(); c++) {
		coords->point.set1Value(c, mesh->getVertex(c).coords[0], mesh->getVertex(c).coords[1], mesh->getVertex(c).coords[2]);
		mat->diffuseColor.set1Value(c, mesh->getVertex(c).color[0], mesh->getVertex(c).color[1], mesh->getVertex(c).color[2]);
	}

	SoMaterialBinding* materialBinding = new SoMaterialBinding; //for 2+ diffuse color usage on the same mesh
	materialBinding->value = SoMaterialBinding::PER_VERTEX_INDEXED;
	res->addChild(materialBinding);

	SoIndexedFaceSet* faceSet = new SoIndexedFaceSet();
	int start_index = 0;
	for (int c = 0; c < mesh->getNumOfTris(); c++)
	{
		faceSet->coordIndex.set1Value(start_index + c * 4, mesh->getTriangle(c).corners[0]);
		faceSet->coordIndex.set1Value(start_index + c * 4 + 1, mesh->getTriangle(c).corners[1]);
		faceSet->coordIndex.set1Value(start_index + c * 4 + 2, mesh->getTriangle(c).corners[2]);
		faceSet->coordIndex.set1Value(start_index + c * 4 + 3, -1);

		faceSet->materialIndex.set1Value(0 + 4 * c, mesh->getTriangle(c).corners[0]);
		faceSet->materialIndex.set1Value(1 + 4 * c, mesh->getTriangle(c).corners[1]);
		faceSet->materialIndex.set1Value(2 + 4 * c, mesh->getTriangle(c).corners[2]);

	}

	res->addChild(coords);
	res->addChild(faceSet);
}

void Painter::drawTriangulation(Mesh* mesh, SoSeparator* res) {

	SoSeparator* thickEdgeSep = new SoSeparator;
	SoMaterial* ma = new SoMaterial;
	ma->diffuseColor.set1Value(1, 0.5, 0.5, 0.5);
	ma->transparency = 0.9;
	thickEdgeSep->addChild(ma);
	SoDrawStyle* sty = new SoDrawStyle;
	sty->lineWidth = 0.5f;
	thickEdgeSep->addChild(sty);

	SoIndexedLineSet* ils = new SoIndexedLineSet;
	SoCoordinate3* co = new SoCoordinate3;
	for (int se = 0; se < mesh->getNumOfEdges(); se++)
	{
		co->point.set1Value(2 * se, mesh->getVertex(mesh->getEdge(se).endVerts[0]).coords[0],
									mesh->getVertex(mesh->getEdge(se).endVerts[0]).coords[1], 
									mesh->getVertex(mesh->getEdge(se).endVerts[0]).coords[2]);
		co->point.set1Value(2 * se + 1, mesh->getVertex(mesh->getEdge(se).endVerts[1]).coords[0],
										mesh->getVertex(mesh->getEdge(se).endVerts[1]).coords[1], 
										mesh->getVertex(mesh->getEdge(se).endVerts[1]).coords[2]);
	}

	for (int ci = 0; ci < mesh->getNumOfEdges(); ci++)
	{
		ils->coordIndex.set1Value(3 * ci, 2 * ci);
		ils->coordIndex.set1Value(3 * ci + 1, 2 * ci + 1);
		ils->coordIndex.set1Value(3 * ci + 2, -1);
	}

	thickEdgeSep->addChild(co);
	thickEdgeSep->addChild(ils);
	res->addChild(thickEdgeSep);

}
