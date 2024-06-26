// @author Merve Asiler

#include "KernelComputation.h"
#include "Parameters.h"

int main()
{

	string command_type = "", input_path = "", output_path = "", input_folder = "", output_folder = "";
	string number_of_rays = "", recursion_depth_limit = "", ray_distribution_type = "";

	/* Example:
	command_type = "approx_ker";
	input_path = "D:/VS_Workspace/3D_Databases/DB-Star-shaped-meshes/data/star.off";
	output_path = "D:/VS_Workspace/3D_Databases/DB-Star-shaped-meshes/data/star" or "d";
	input_folder = "D:/VS_Workspace/3D_Databases/DB-Thingi/data";
	output_folder = "D:/VS_Workspace/3D_Databases/DB-Thingi/KernelResults-";
	*/

	// Command type
	cout << "Enter the command type: ";			cin >> command_type;		cout << endl;;

	// Single Execution
	if (command_type == "approx_ker" ||							// the proposed algorithm
		command_type == "kernel_by_cgal" ||						// CGAL's kernel computation algorithm
		command_type == "compare_algos") {						// compares the proposed algorithm with CGAL's

		cout << "Enter the input path (.off/.obj): ";		cin >> input_path;		cout << endl;

		if (command_type == "approx_ker" || command_type == "kernel_by_cgal") {
			cout << "Enter the output path : ";		cin >> output_path;		cout << endl;
		}
	}

	// Batch Execution
	else if (command_type == "batch_approx_ker" ||
		command_type == "batch_kernel_by_cgal") {

		cout << "Enter the input folder: ";			cin >> input_folder;		cout << endl;
		cout << "Enter the output folder: ";		cin >> output_folder;		cout << endl;
	}

	// Undefined Operation
	else
		cout << "Undefined Operation!" << endl;


	// Define parameter values for the proposed approximate kernel algorithm
	if (command_type == "approx_ker" || "batch_approx_ker" || "compare_algos")
		SetParameters();


	cout << "Processing..." << endl << endl;


	// COMPUTE KERNEL BY <"KERNEL APPROXIMATION"  OR  "CGAL">
	if (command_type == "approx_ker" || command_type == "kernel_by_cgal")
		ComputeKernel(input_path, output_path, command_type);

	// COMPUTE KERNEL BY <"KERNEL APPROXIMATION"  OR  "CGAL"> FOR ALL MESH FILES IN THE GIVEN FOLDER
	else if (command_type == "batch_approx_ker" || command_type == "batch_kernel_by_cgal")
		ComputeBatchKernel(input_folder, output_folder, command_type);

	// COMPARE KERNEL RESULTS for "CGAL"  AND  "KERNEL APPROXIMATION"
	else // if (command_type == "compare_algos")
		CompareAlgos(input_path);


	// Finalize
	cout << "Completed." << endl;

	return 0;

}
