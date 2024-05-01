// @author Merve Asiler

#include "KernelComputation.h"
#include "KernelApproximation.h"
#include "KernelByCGAL.h"

#include "sdlp.h"
#include "SceneManager.h"
#include "CommonUtils.h"
#include "CGALUtils.h"

#include <boost/filesystem.hpp>
#include <ctime>
#include <fstream>

using namespace std;
using namespace boost::filesystem;

/* ***************************** GLOBAL VARIABLES ***************************** */
/* */ string algoVersionType = "none";
/* */ double executionCount = 1;
/* */ bool qualityComparison = true;
/* */ double _MAX_DISTANCE_ = -numeric_limits<double>::infinity();
/* */ vector<double> _DISTANCES_;
/* **************************************************************************** */

void ComputeKernel(string meshName, string algoType) {

	// Read mesh
	Mesh mesh;
	if (meshName.substr(meshName.length() - 3, 3) == "off")
		mesh.loadOff(meshName.c_str());
	else
		mesh.loadObj(meshName.c_str());

	// Ingredients:
	double elapsedTime = 0;
	double volDiffPercentage = 0;
	double hausdorffDistances[3] = { 0, 0, 0 };
	Mesh kernel;
	string tempFileName = "temp_file.txt";
	std::ofstream outputFile(tempFileName);

	// Choose the method
	vector<KernelExpansion*> kernelExpansions;
	for (int i = 0; i < executionCount; i++) {
		if (algoType == "approx_ker")
			kernelExpansions.push_back(new KernelApproximation(mesh));
		else if (algoType == "kernel_by_cgal")
			kernelExpansions.push_back(new KernelByCGAL(mesh));
		else;
	}

	// Run
	kernel = Run(kernelExpansions, mesh, outputFile, elapsedTime, algoType);

	// Quality
	if (qualityComparison && kernel.getNumOfVerts() > 0) {
		KernelByCGAL kernelByCGAL(mesh);
		kernelByCGAL.expandKernel();
		Mesh groundTruth = kernelByCGAL.getKernel();
		CompareKernelQuality(groundTruth, kernel, algoType, outputFile, volDiffPercentage, hausdorffDistances);
	}

	// Print
	outputFile.close();
	std::ifstream inputFile(tempFileName);
	std::stringstream ss;
	ss << inputFile.rdbuf();
	cout << ss.str();
	inputFile.close();
	remove(tempFileName);

	// Draw
	if (kernel.getNumOfVerts() > 0) {
		MaterialSetting* kernelMatSetting = new MaterialSetting(0, 0, 1, 0);
		MaterialSetting* meshMatSetting = new MaterialSetting(1, 1, 1, 0.5);
		vector<tuple<Mesh*, MaterialSetting*>> mesh_mat_set = { make_tuple(&kernel, kernelMatSetting), make_tuple(&mesh, meshMatSetting) };
		DrawMultipleMeshToScene(mesh_mat_set);
	}

}

void ComputeBatchKernel(string inputFolderName, string outputFolderName, string algoType) {

	// Detect the algo type
	string extension, statistics_file_name;
	statistics_file_name = "KernelResults_" + algoType + ".txt";
	extension = "_" + algoType + "_kernel.off";

	double avgTime_star = 0, avgTime_nonstar = 0;
	int numOfStarShapes = 0, numOfNonStarShapes = 0, numOfNonManifoldShapes = 0;
	double avgVolDiffPercentage = 0;				// for star-shapes only
	double avgHausdorffDistances[3] = { 0, 0, 0 };	// for star-shapes only

	// Read folder, fecth mesh names, compute kernels
	int mesh_rank_id = 0;
	path p(inputFolderName);
	for (auto i = directory_iterator(p); i != directory_iterator(); i++, mesh_rank_id++)
	{
		// Fetch the mesh name
		string meshName;
		if (!is_directory(i->path())) //we eliminate directories
			meshName = i->path().filename().string();
		else
			continue;

		// if this is a previously created kernel file
		if (meshName.length() > 11 && meshName.substr(meshName.length() - 11, 11) == "_kernel.off")
			continue;

		// Open the file to append statistics & results
		std::ofstream outputFile(outputFolderName + "/" + statistics_file_name, ios_base::app);
		cout << meshName << endl;

		// Read mesh
		Mesh mesh;
		if (meshName.substr(meshName.length() - 3, 3) == "off")
			mesh.loadOff((inputFolderName + "/" + meshName).c_str());
		else
			mesh.loadObj((inputFolderName + "/" + meshName).c_str());

		if (mesh.isManifold() == false) {
			outputFile << "NOT MANIFOLD: " << mesh_rank_id << ": " << meshName << " !" << endl;
			cout << "Not manifold: " << mesh_rank_id << ": " << meshName << " !" << endl;
			numOfNonManifoldShapes++;
			continue;
		}


		cout << "Processing: " << mesh_rank_id << ": " << meshName << endl;
		outputFile << meshName << endl;

		// Choose the method
		double elapsedTime = 0;
		Mesh kernel;
		vector<KernelExpansion*> kernelExpansions;
		for (int i = 0; i < executionCount; i++) {
			if (algoType == "batch_approx_ker")
				kernelExpansions.push_back(new KernelApproximation(mesh));
			else if (algoType == "batch_kernel_by_cgal")
				kernelExpansions.push_back(new KernelByCGAL(mesh));
			else;
		}

		// Run
		kernel = Run(kernelExpansions, mesh, outputFile, elapsedTime, algoType);

		// Quality
		if (qualityComparison && kernel.getNumOfVerts() > 0) {
			KernelByCGAL kernelByCGAL(mesh);
			kernelByCGAL.expandKernel();
			Mesh groundTruth = kernelByCGAL.getKernel();
			double volDiffPercentage;
			double hausdorffDistances[3];
			CompareKernelQuality(groundTruth, kernel, algoType, outputFile, volDiffPercentage, hausdorffDistances);
			avgVolDiffPercentage += volDiffPercentage;
			for (int k = 0; k < 3; k++)
				avgHausdorffDistances[k] += hausdorffDistances[k];
		}

		// Output notes
		if (kernel.getNumOfVerts() > 0) {
			kernel.writeOff(outputFolderName + "/" + meshName.substr(0, meshName.length() - 4) + extension);
			avgTime_star += elapsedTime;
			numOfStarShapes++;
		}
		else {
			avgTime_nonstar += elapsedTime;
			numOfNonStarShapes++;
		}

		outputFile.close();
	}

	std::ofstream outputFile(outputFolderName + "/" + statistics_file_name, ios_base::app);
	outputFile << endl << endl;
	outputFile << "AVERAGE kernel computation has been completed in " << avgTime_star / numOfStarShapes << " milisecond(s) for " << numOfStarShapes << " star-shapes." << endl;
	outputFile << "AVERAGE kernel computation has been completed in " << avgTime_nonstar / numOfNonStarShapes << " milisecond(s) for " << numOfNonStarShapes << " nonstar-shapes." << endl;
	outputFile << "AVERAGE percentage of volume differences in computed kernels is " << avgVolDiffPercentage / numOfStarShapes << "% for " << numOfStarShapes << " star-shapes." << endl;
	outputFile << "AVERAGE hausdorff distance as 'grountTruth -> computed' is " << avgHausdorffDistances[0] / numOfStarShapes << " for " << numOfStarShapes << " star-shapes." << endl;
	outputFile << "AVERAGE hausdorff distance as 'computed -> grountTruth' is " << avgHausdorffDistances[1] / numOfStarShapes << " for " << numOfStarShapes << " star-shapes." << endl;
	outputFile << "AVERAGE hausdorff distance as 'symmetric' is " << avgHausdorffDistances[2] / numOfStarShapes << " for " << numOfStarShapes << " star-shapes." << endl;
	outputFile << "There has not been done kernel computation operation for " << numOfNonManifoldShapes << " non-manifold shapes." << endl;
	outputFile.close();

}

Mesh Run(vector<KernelExpansion*>& kernelExpansions, Mesh& mesh, std::ofstream& outputFile, double& elapsedTime, string algoType) {

	// Execute by <executionCount>-many times
	Mesh kernel;
	double totalTime = 0;
	bool applyColorMap = false;
	if (algoVersionType.find("colormap") != string::npos) {
		algoVersionType = algoVersionType.substr(0, algoVersionType.find("colormap"));
		applyColorMap = true;
	}

	for (int i = 0; i < kernelExpansions.size(); i++) {
		KernelExpansion* kernelExpansion = kernelExpansions[i];
		clock_t begin = clock();
		kernelExpansion->expandKernel();
		kernel = kernelExpansion->getKernel();
		clock_t end = clock();
		totalTime += double(end - begin) * 1000.0 / CLOCKS_PER_SEC;
		mesh.setKernelAABB(kernelExpansion->extremeCorners[0], kernelExpansion->extremeCorners[1]);
		delete kernelExpansion;
	}

	// Print the statistics
	outputFile << "Mesh: [faces: " << mesh.getNumOfTris() << "], [edges: " << mesh.getNumOfEdges() << "], [vertices: " << mesh.getNumOfVerts() << "]" << endl;
	if (kernel.getNumOfVerts() > 0)
		outputFile << "Kernel: [faces: " << kernel.getNumOfTris() << "], [edges: " << kernel.getNumOfEdges() << "], [vertices: " << kernel.getNumOfVerts() << "]" << endl;
	else
		outputFile << "Kernel is empty!" << endl;

	elapsedTime = totalTime / executionCount;
	outputFile << "Kernel computation has been completed in " << elapsedTime << " milisecond(s) by " << algoType << " with the followings parameters : \n";

	if (applyColorMap) {
		KernelExpansion* kernelExpansion = new KernelByCGAL(mesh);
		kernelExpansion->expandKernel();
		Mesh groundTruth = kernelExpansion->getKernel();
		delete kernelExpansion;
		produceColorSource(groundTruth, kernel);
		return groundTruth;
	}
	return kernel;

}

void CompareKernelQuality(Mesh groundTruth, Mesh kernel, string algoType, std::ofstream& outputFile, double& volDiffPercentage, double* hausdorffDistances) {

	double groundTruthVolume = groundTruth.computeVolume();
	double volume = 0;
	if (kernel.getNumOfVerts() > 0)
		volume = kernel.computeVolume();
	if (volume < EPSILON)
		outputFile << "comparison is invalid due to ZERO volume!" << endl;

	outputFile << "Volume of the kernel computed by <" << algoType << "> : " << volume << " out of " << groundTruthVolume << "." << endl;
	double volumeDiff = groundTruthVolume - volume;
	if (abs(volumeDiff) < EPSILON)
		volumeDiff = 0;
	volDiffPercentage = (100.0 * abs(volumeDiff)) / groundTruthVolume;
	outputFile << ">>>> Volume difference : " << volumeDiff << endl;
	outputFile << ">>>> Percentage of difference : " << volDiffPercentage << "%" << endl;

	double* hd = computeHausdorffDistance(groundTruth, kernel);

	for (int k = 0; k < 3; k++) {
		if (hd[k] < EPSILON)
			hd[k] = 0;
		hausdorffDistances[k] = hd[k];
	}
	outputFile << "Hausdorff distances : " << endl;
	outputFile << "\t\t    grountTruth -> computed : " << hd[0] << endl;
	outputFile << "\t\t    computed -> groundTruth : " << hd[1] << endl;
	outputFile << "\t\t    symmetric : " << hd[2] << " ." << endl << endl;
	delete[] hd;

}

void FindKernelPoint_SDLP(string meshName) {

	Mesh mesh;
	if (meshName.substr(meshName.length() - 3, 3) == "off")
		mesh.loadOff(meshName.c_str());
	else
		mesh.loadObj(meshName.c_str());

	double extremeDirection[3] = { 0, 0, 1 };

	double* kernel_point = sdlpMain(mesh, extremeDirection);
	if (kernel_point != NULL) {
		cout << "Final kernel point: " << kernel_point[0] << " " << kernel_point[1] << " " << kernel_point[2] << endl;
		delete[] kernel_point;
	}
}

void DoVisualComparisonOfAlgos(string meshName) {

	/************************************************ READ MESH ************************************************/
	Mesh mesh;
	if (meshName.substr(meshName.length() - 3, 3) == "off")
		mesh.loadOff(meshName.c_str());
	else
		mesh.loadObj(meshName.c_str());

	/*************************************** INGREDIENTS / PREPARATIONS  ***************************************/
		// ... for kernel computation

	vector<string> algoTypes{ "kernel_by_cgal", "kernel_by_cgal", "approx_ker", "approx_ker" };
	vector<string> versionTypes{ "none", "none", "none", "colormap"};
	
	vector<Mesh> kernels;
	kernels.resize(algoTypes.size());
	// ... for positions of the shapes on the scene
	vector<double*> positions;
	// compute the bounding box of the shape
	double minborder[3], maxborder[3];
	for (int j = 0; j < 3; j++) {
		minborder[j] = numeric_limits<double>::infinity();
		maxborder[j] = -numeric_limits<double>::infinity();
	}
	for (int i = 0; i < mesh.getNumOfVerts(); i++) {	// find the "most" and "least" coordinates of the mesh
		Vertex vertex = mesh.getVertex(i);
		for (int j = 0; j < 3; j++) {
			if (minborder[j] > vertex.coords[j])
				minborder[j] = vertex.coords[j];
			if (maxborder[j] < vertex.coords[j])
				maxborder[j] = vertex.coords[j];
		}
	}
	// define scene size using bounding box
	double objectWidth[3], sceneSizeUnit = 0;
	for (int j = 0; j < 3; j++) {
		objectWidth[j] = maxborder[j] - minborder[j];
		if (objectWidth[j] > sceneSizeUnit)
			sceneSizeUnit = objectWidth[j];
	}
	double totalSceneSize = (sceneSizeUnit * 2 * algoTypes.size());
	// ... for shape materials
	vector<tuple<tuple<Mesh*, MaterialSetting*>, tuple<Mesh*, MaterialSetting*>>> mesh_mat_sets;
	MaterialSetting* kernelMatSetting_blue = new MaterialSetting(0, 0, 1, 0);
	MaterialSetting* kernelMatSetting_white = new MaterialSetting(1, 1, 1, 0);
	MaterialSetting* meshMatSetting = new MaterialSetting(1, 1, 1, 0.9);
	// ... for printing results&statistics
	string tempFileName = "temp_file.txt";
	std::ofstream outputFile(tempFileName);

	/************************************************ ALGORITHMS ************************************************/

	for (int i = 0; i < algoTypes.size(); i++) {
		vector<KernelExpansion*> kernelExpansions;
		string algoType = algoTypes[i];
		algoVersionType = versionTypes[i];
		double elapsedTime = 0;
		double volDiffPercentage = 0;
		double hausdorffDistances[3] = { 0, 0, 0 };

		// Choose algoType
		if (algoType == "approx_ker")
			kernelExpansions.push_back(new KernelApproximation(mesh));
		else if (algoType == "kernel_by_cgal")
			kernelExpansions.push_back(new KernelByCGAL(mesh));
		else;

		// Run
		if (i < 3)
			kernels[i] = Run(kernelExpansions, mesh, outputFile, elapsedTime, algoType);
		else { // i == 3
			kernels[i] = kernels[1];	// copy
			computeDistancesForError(kernels[i], kernels[i-1]);
			produceColorSource(kernels[i]);
		}

		// Material upload
		if (i < 2)
			mesh_mat_sets.push_back(make_tuple(make_tuple(&kernels[i], kernelMatSetting_blue), make_tuple(&mesh, meshMatSetting)));
		else
			mesh_mat_sets.push_back(make_tuple(make_tuple(&kernels[i], kernelMatSetting_white), make_tuple(&mesh, meshMatSetting)));

		// Position upload
		positions.push_back(new double[3]{ 1.5 * i * sceneSizeUnit, 0, 0 });
		
		// Quality
		if (qualityComparison && kernels[i].getNumOfVerts() > 0 && i > 1)
			CompareKernelQuality(kernels[0], kernels[i], algoType, outputFile, volDiffPercentage, hausdorffDistances);
		kernelExpansions.clear();
	}

	// Print
	outputFile.close();
	std::ifstream inputFile(tempFileName);
	std::stringstream ss;
	ss << inputFile.rdbuf();
	cout << ss.str();
	inputFile.close();
	remove(tempFileName);

	/*************************************************** DRAW ***************************************************/
	if (kernels[0].getNumOfVerts() > 0)
		DrawMultipleScenes(mesh_mat_sets, positions, totalSceneSize);

	/************************************************* CLEAN-UP *************************************************/
	for (int i = 0; i < algoTypes.size(); i++)
		delete[] positions[i];
	delete kernelMatSetting_blue;
	delete kernelMatSetting_white;
	delete meshMatSetting;

}

vector<double> produceColorSource(Mesh& ground_truth, Mesh& exp_mesh) {

	vector<double> distances;
	double maxDistance = -numeric_limits<double>::infinity();
	double color[3] = { 1.0, 1.0, 1.0 };

	for (int i = 0; i < ground_truth.getNumOfVerts(); i++) {

		Vertex v1 = ground_truth.getVertex(i);
		double minDistance = numeric_limits<double>::infinity();

		for (int j = 0; j < exp_mesh.getNumOfVerts(); j++) {

			Vertex v2 = exp_mesh.getVertex(j);
			double* diff = diffVects(v1.coords, v2.coords);
			double distance = computeLength(diff);
			delete[] diff;

			if (distance < minDistance)
				minDistance = distance;
		}

		if (minDistance < EPSILON)
			minDistance = 0;
		distances.push_back(minDistance);
		if (minDistance > maxDistance)
			maxDistance = minDistance;

		ground_truth.addVertexColor(i, color);
	}

	if (!(maxDistance < EPSILON)) {
		// normalize
		for (int i = 0; i < distances.size(); i++) {
			//distances[i] /= maxDistance;

			if (distances[i] > 0.5) {
				color[0] = 1.0;
				color[1] = abs(1.0 - distances[i]) * 2;
				color[2] = 0; // abs(0.5 - abs(2 * (1.0 - 2 * distances[i]) + 1.0) / 2.0);	// 0'dan 0'a  max: 0.5
			}
			else {
				color[0] = 1.0;
				color[1] = 1.0;
				color[2] = abs(1.0 - 2 * distances[i]);		// 0'dan (0.5 iken) 1'e ( 0 iken) 
			}

			if (distances[i] < EPSILON) {
				for (int k = 0; k < 3; k++)
					color[k] = 1.0;
			}

			ground_truth.addVertexColor(i, color);
		}
	}

	return distances;
}

void produceColorSource(Mesh& ground_truth) {

	double color[3] = { 1.0, 1.0, 1.0 };

	if (!(_MAX_DISTANCE_ < EPSILON)) {
		// normalize
		for (int i = 0; i < _DISTANCES_.size(); i++) {
			_DISTANCES_[i] /= _MAX_DISTANCE_;

			if (_DISTANCES_[i] > 0.5) {
				color[0] = 1.0;
				color[1] = abs(1.0 - _DISTANCES_[i]) * 2;
				color[2] = 0; // abs(0.5 - abs(2 * (1.0 - 2 * distances[i]) + 1.0) / 2.0);	// 0'dan 0'a  max: 0.5
			}
			else {
				color[0] = 1.0;
				color[1] = 1.0;
				color[2] = abs(1.0 - 2 * _DISTANCES_[i]);		// 0'dan (0.5 iken) 1'e ( 0 iken) 
			}

			if (_DISTANCES_[i] < EPSILON) {
				for (int k = 0; k < 3; k++)
					color[k] = 1.0;
			}

			ground_truth.addVertexColor(i, color);
		}
	}

}

void computeDistancesForError(Mesh& ground_truth, Mesh& exp_mesh) {

	double color[3] = { 1.0, 1.0, 1.0 };

	for (int i = 0; i < ground_truth.getNumOfVerts(); i++) {

		Vertex v1 = ground_truth.getVertex(i);
		double minDistance = numeric_limits<double>::infinity();

		for (int j = 0; j < exp_mesh.getNumOfVerts(); j++) {

			Vertex v2 = exp_mesh.getVertex(j);
			double* diff = diffVects(v1.coords, v2.coords);
			double distance = computeLength(diff);
			delete[] diff;

			if (distance < minDistance)
				minDistance = distance;
		}

		if (minDistance < EPSILON)
			minDistance = 0;
		_DISTANCES_.push_back(minDistance);
		if (minDistance > _MAX_DISTANCE_)
			_MAX_DISTANCE_ = minDistance;

		ground_truth.addVertexColor(i, color);
	}

}

