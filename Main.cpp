// @author Merve Asiler

#include "KernelComputation.h"
#include "SceneManager.h"

int main()
{

	string command_type = "", shape_path = "", input_folder = "", output_folder = "";

	// Command type
	cout << "Enter the command type: ";			cin >> command_type;		cout << endl;;

	// Single Execution
	if (command_type == "draw" ||
		command_type == "approx_ker" ||
		command_type == "kernel_by_cgal" ||
		command_type == "visual_comparison_of_algos" ||
		command_type == "findkernelpoint_SDLP") {

		cout << "Enter the shape path (.off/.obj): ";		cin >> shape_path;		cout << endl;
	}

	// Batch Execution
	else if (command_type == "batch_approx_ker" ||
			 command_type == "batch_kernel_by_cgal") {

		cout << "Enter the input folder: ";			cin >> input_folder;		cout << endl;
		cout << "Enter the output folder: ";		cin >> output_folder;		cout << endl;
		output_folder += command_type;
	}

	// Undefined Operation
	else
		cout << "Undefined Operation!" << endl;

	/* Example:
		command_type = "approx_ker";
		shape_path = "D:/VS_Workspace/3D_Databases/DB-Star-shaped-meshes/data/star.off";
		input_folder = "D:/VS_Workspace/3D_Databases/DB-Thingi/data";
		output_folder = "D:/VS_Workspace/3D_Databases/DB-Thingi/KernelResults-";
	*/




	// DRAW:
	if (command_type == "draw")
		drawMeshToScene(shape_path);

	// COMPUTE KERNEL BY <"KERNEL APPROXIMATION"  OR  "CGAL">
	else if (command_type == "approx_ker" || command_type == "kernel_by_cgal")
		ComputeKernel(shape_path, command_type);

	// COMPUTE KERNEL BY <"KERNEL APPROXIMATION"  OR  "CGAL"> FOR ALL MESH FILES IN THE GIVEN FOLDER
	else if (command_type == "batch_approx_ker" || command_type == "batch_kernel_by_cgal")
		ComputeBatchKernel(input_folder, output_folder, command_type);

	// COMPARE KERNEL RESULTS for "CGAL"  AND  "KERNEL APPROXIMATION"
	else if (command_type == "visual_comparison_of_algos")
		doVisualComparisonOfAlgos(shape_path);

	// FIND A KERNEL POINT MAXIMIZING A STATED COST FUNCTION by THIRD PARTY LIBRARY : SDLP
	else
		FindKernelPoint_SDLP(shape_path);


	// Finalize
	cout << "Completed." << endl;

	return 0;

}
