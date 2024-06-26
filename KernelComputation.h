// @author Merve Asiler

#pragma once
#pragma comment(lib, "boost_filesystem-vc140-mt.lib")

#include <string>
#include <vector>
using namespace std;

class Mesh;
class KernelExpansion;

/*!
 *	@function			ComputeKernel	
 *	@abstract			manages the operations to compute the kernel for a given mesh	
 *	@param	meshName	the path of the given mesh file (.obj or .off format)
 *	@param	outputName	the path of the output mesh file for the kernel
 *	@param	algoType	the proposed approximation algorithm or CGAL's kernel computation algorithm
 *	@result				if the kernel exists, it is written to a file and drawn to the screen
*/
void ComputeKernel(string meshName, string outputName, string algoType);

/*!
 *	@function					ComputeBatchKernel
 *	@abstract					manages the operations to compute the kernel for the meshes given in a folder
 *	@param	inputFolderName		the path of the folder containing the meshes (.obj or .off format)
 *	@param	outputFolderName	the path of the output mesh file for the kernel
 *	@param	algoType			the proposed approximation algorithm or CGAL's kernel computation algorithm
 *	@result						for meshes having a non-empty kernel, their kernel meshes are written to a file
*/
void ComputeBatchKernel(string inputFolderName, string outputFolderName, string algoType);

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
Mesh Run(vector<KernelExpansion*>& kernelExpansions, Mesh& mesh, std::ofstream& outputFile, double& elapsedTime, string algoType);

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
void CompareKernelQuality(Mesh groundTruth, Mesh kernel, string algoType, std::ofstream& outputFile, double& volDiffPercentage, double* hausdorffDistances);

/*!
 *	@function	FindKernelPoint_SDLP
 *	@abstract	find the extreme kernel point for the mesh in the path <meshName>
*/
void FindKernelPoint_SDLP(string meshName);

/*!
 *	@function	CompareAlgos
 *	@abstract	compare the kernels computed by the proposed approximation algorithm and CGAL for the mesh in the path <meshName>
*/
void CompareAlgos(string meshName);

/*!
 *	@function	produceColorSource
 *	@abstract	color the vertices of the <ground_truth> based on the error rates of the <exp_mesh> measured by the Eucledean distance.
*/
vector<double> produceColorSource(Mesh& ground_truth, Mesh& exp_mesh);

/*!
 *	@function	produceColorSource
	@abstract	color the vertices of the <ground_truth> based on the error rates measured by the Eucledean distance.
 *				distances which represent the errors are given global.
*/
void produceColorSource(Mesh& ground_truth);

/*!
 *	@function	computeDistancesForError
 *	abstract	compute error rates measuring the Eucledean distances between each corresponding vertex of <ground_truth> and <exp_mesh>.
*/
void computeDistancesForError(Mesh& ground_truth, Mesh& exp_mesh);




