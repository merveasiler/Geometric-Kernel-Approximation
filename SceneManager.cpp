// @author Merve Asiler

#include "SceneManager.h"
#include "CommonUtils.h"

#include <string>

void DrawMeshToScene(string meshName) {

	Mesh* mesh = new Mesh;
	if (meshName.substr(meshName.length() - 3, 3) == "off")
		mesh->loadOff(meshName.c_str());
	else
		mesh->loadObj(meshName.c_str());

	Scene* scene = new Scene();
	Painter* painter = new Painter();
	SoSeparator* res = new SoSeparator();
	painter->getShapeSep(mesh, res);
	painter->drawTriangulation(mesh, res);
	scene->makeScene(res);
	delete scene;
	delete painter;

	delete mesh;

}

void DrawMultipleMeshToScene(vector<tuple<Mesh*, MaterialSetting*>> mesh_mat_set) {

	Scene* scene = new Scene();
	Painter* painter = new Painter();

	vector<SoSeparator*> resSet;
	for (int i = 0; i < mesh_mat_set.size(); i++) {
		Mesh* mesh;
		MaterialSetting* mat;
		tie(mesh, mat) = mesh_mat_set[i];
		SoSeparator* res = new SoSeparator();
		
		SoTransform* transform_r = new SoTransform();
		SbVec3f axis(0, 1, 0);
		transform_r->rotation.setValue(axis, PI / 5);
		res->addChild(transform_r);

		if (i == 0)
			painter->getShapeSep(mesh, mat, res);
		if (i > 0)
			painter->drawTriangulation(mesh, res);

		resSet.push_back(res);
	}

	scene->makeScene(resSet);

	resSet.clear();
	delete scene;
	delete painter;

}

void DrawMultipleScenes(vector<tuple<tuple<Mesh*, MaterialSetting*>, tuple<Mesh*, MaterialSetting*>>> outputs, vector<double*> positions, double sceneSize) {

	Scene* scene = new Scene();
	vector<SoSeparator*> resSets;
	SoSeparator* cornerResSet = NULL;
	Painter* painter = new Painter();
	
	// SCENES
	for (int i = 0; i < outputs.size(); i++) {
		tuple<tuple<Mesh*, MaterialSetting*>, tuple<Mesh*, MaterialSetting*>> kernel_mesh_tuple = outputs[i];
		tuple<Mesh*, MaterialSetting*> kernel_mat_set, mesh_mat_set;
		tie(kernel_mat_set, mesh_mat_set) = kernel_mesh_tuple;

		Mesh* mesh[2];
		MaterialSetting* mesh_mat[2];
		tie(mesh[0], mesh_mat[0]) = kernel_mat_set;
		tie(mesh[1], mesh_mat[1]) = mesh_mat_set;

		SoSeparator* resSet = new SoSeparator;
		resSets.push_back(resSet);

		// compute position
		SoTransform* transform = new SoTransform();
		transform->translation.setValue(positions[i][0], positions[i][1], positions[i][2]);
		resSet->addChild(transform);

		SoTransform* transform_r = new SoTransform();
		SbVec3f axis(0, 1, 0);
		transform_r->rotation.setValue(axis, 5 * PI / 6);
		//resSet->addChild(transform_r);

		for (int j = 0; j < 2; j++) {
			SoSeparator* res = new SoSeparator();
			if (j == 0) {
				if (i == 3)
					painter->getColorfulShapeSep(mesh[j], res);
				else
					painter->getShapeSep(mesh[j], mesh_mat[j], res);
			}
			if (j > 0 && i == 0)
				painter->drawTriangulation(mesh[j], res);
			resSet->addChild(res);
		}
	}
	
	scene->makeMultipleScene(resSets, cornerResSet, sceneSize);

	// clean-up
	for (int i = 0; i < outputs.size()-1; i++)
		resSets[i]->removeAllChildren();
	resSets.clear();
	delete painter;
	delete scene;

}
