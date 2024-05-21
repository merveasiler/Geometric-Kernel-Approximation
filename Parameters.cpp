// @author Merve Asiler

#include "Parameters.h"

void setNumOfRays(string value) {

	if (value == "d" || value == "D")
		;
	else
		number_of_rays = stoi(value);
}

void setRecDepLim(string value) {

	if (value == "d" || value == "D")
		;
	else
		recursion_depth_limit = stoi(value);
}

void setRayDistType(string value) {

	if (value == "d" || value == "D" || value == "geometry-based")
		;
	else
		ray_distribution_type = "spherical-based";
}

void SetParameters() {

	string value = "";

	cout << "Enter the number of rays [default -> 30] : ";						cin >> value;	setNumOfRays(value);	cout << endl;

	cout << "Enter the recursion depth limit [default -> 3] : ";				cin >> value;	setRecDepLim(value);	cout << endl;

	cout << "Enter the ray distribution type [default -> geometry-based] : ";	cin >> value;	setRayDistType(value);	cout << endl;

}


