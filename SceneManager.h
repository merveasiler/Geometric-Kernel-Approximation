// @author Merve Asiler

#pragma once

#include "Painter.h"
#include "Scene.h"
#include "Mesh.h"

// SCENE DRAWING MANAGER METHODs

void drawMeshToScene(string meshName);

void drawMultipleMeshToScene(vector<tuple<Mesh*, MaterialSetting*>> mesh_mat_set);

void drawMultipleScenes(vector<tuple<tuple<Mesh*, MaterialSetting*>, tuple<Mesh*, MaterialSetting*>>> outputs, vector<double*> positions, double sceneSize);


