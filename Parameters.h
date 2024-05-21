// @author Merve Asiler

#pragma once

#include <iostream>
#include <string>

using namespace std;

// Parameters for the Approximate Kernel Computation Algorithm

static int number_of_rays = 30;
static int recursion_depth_limit = 3;
static string ray_distribution_type = "geometry-based";

void setNumOfRays(string value);

void setRecDepLim(string value);

void setRayDistType(string value);

void SetParameters();




