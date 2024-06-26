// @author Merve Asiler

#include "KernelComputation.h"
#include "KernelApproximation.h"
#include "KernelByCGAL.h"

#include "sdlp.h"
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

/*!
 *	@function			ComputeKernel
 *	@abstract			manages the operations to compute the kernel for a given mesh
 *	@param	meshName	the path of the given mesh file (.obj or .off format)
 *	@param	outputName	the path of the output mesh file for the kernel
 *	@param	algoType	the proposed approximation algorithm or CGAL's kernel computation algorithm
 *	@result				if the kernel exists, it is written to a file and drawn to the screen
*/
void ComputeKernel(string meshName, string outputName, string algoType) {

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

	// Write kernel
	if (kernel.getNumOfVerts() > 0) {
		if (outputName == "d" || outputName == "D") {
			string extension = "_kernel_" + algoType + ".off";
			kernel.writeOff(meshName.substr(0, meshName.length() - 4) + extension);
		}
		else
			kernel.writeOff(outputName + ".off");
	}

}

/*!
 *	@function					ComputeBatchKernel
 *	@abstract					manages the operations to compute the kernel for the meshes given in a folder
 *	@param	inputFolderName		the path of the folder containing the meshes (.obj or .off format)
 *	@param	outputFolderName	the path of the output mesh file for the kernel
 *	@param	algoType			the proposed approximation algorithm or CGAL's kernel computation algorithm
 *	@result						for meshes having a non-empty kernel, their kernel meshes are written to a file
*/
void ComputeBatchKernel(string inputFolderName, string outputFolderName, string algoType) {

	// Detect the algo type
	string extension, statistics_file_name;
	statistics_file_name = "KernelResults_" + algoType + ".txt";
	extension = "_" + algoType + "_kernel.off";

	if (outputFolderName == "d" || outputFolderName == "D")
		outputFolderName = inputFolderName;

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

/*!
 *	@function					Run
 *	@abstract					run the kernel computation function for a given mesh
 *	@param	kernelExpansions	constructors to initiate kernel computation for a specified algorithm
 *	@param  mesh				the input mesh whose kernel is to be computed
 *  @param  outputFile			file to include the command-line output such as kernel properties and operation duration
 *  @param	elapsedTime			time spent for the kernel computation
 *  @param	algoType			kernel computation algorithm type
 *  @result						if the kernel exists, it is written to a file, returned as a triangular mesh and the elapsed time is calculated.
*/
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
	outputFile << "Kernel computation has been completed in " << elapsedTime << " milisecond(s) by " << algoType << ". \n\n";

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

/*!
 *	@function					CompareKernelQuality
 *	@abstract					compare the quality of the <kernel> with the <groundTruth> measuring the volume and Hausdorff distance
 *	@param	groundTruth			the mesh which is assumed to be the ground truth
 *	@param  kernel				the mesh which is asked to compare with the fround truth
 *  @param	algoType			kernel computation algorithm type
 *  @param  outputFile			file to include the command-line output such as kernel quality
 *  @param	volDiffPercentage	percentage of the volume difference between the <kernel> and the <groundTruth>
 *  @param  hausdorffDistances	left, right and symmetric hausdorff distances between the <kernel> and the <groundTruth>
*/
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

/*!
 *	@function	FindKernelPoint_SDLP
 *	@abstract	find the extreme kernel point for the mesh in the path <meshName>
*/
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

/*!
 *	@function	CompareAlgos
 *	@abstract	compare the kernels computed by the proposed approximation algorithm and CGAL for the mesh in the path <meshName>
*/
void CompareAlgos(string meshName) {

	/************************************************ READ MESH ************************************************/
	Mesh mesh;
	if (meshName.substr(meshName.length() - 3, 3) == "off")
		mesh.loadOff(meshName.c_str());
	else
		mesh.loadObj(meshName.c_str());

	/*************************************** INGREDIENTS / PREPARATIONS  ***************************************/
		// ... for kernel computation

	vector<string> algoTypes{ "kernel_by_cgal", "approx_ker", "approx_ker" };	// the last one is to apply color-encoding on vertices w.r.t. Eucledean distances from the actual coordinates in the ground truth 
	vector<string> versionTypes{ "none", "none", "none", "colormap"};
	
	vector<Mesh> kernels;
	kernels.resize(algoTypes.size());

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
		if (i < 2)
			kernels[i] = Run(kernelExpansions, mesh, outputFile, elapsedTime, algoType);
		else { // i == 2
			kernels[i] = kernels[0];	// copy
			computeDistancesForError(kernels[i], kernels[i-1]);
			produceColorSource(kernels[i]);
		}
		
		// Quality
		if (qualityComparison && kernels[i].getNumOfVerts() > 0 && i == 1)
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

}

/*!
 *	@function	produceColorSource
 *	@abstract	color the vertices of the <ground_truth> based on the error rates of the <exp_mesh> measured by the Eucledean distance.
*/
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

/*!
 *	@function	produceColorSource
	@abstract	color the vertices of the <ground_truth> based on the error rates measured by the Eucledean distance.
 *				distances which represent the errors are given global.
*/
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

/*!
 *	@function	computeDistancesForError
 *	abstract	compute error rates measuring the Eucledean distances between each corresponding vertex of <ground_truth> and <exp_mesh>.
*/
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

