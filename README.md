This is the source code for our paper "[3D Geometric Kernel Computation in Polygon Mesh Structures.](https://www.sciencedirect.com/science/article/pii/S0097849324000864)" <br />

We introduce a novel approach to compute the geometric kernel of a polygon mesh embedded in 3D. <br />

If you use this code please give reference to the above publication: <br /> 
--> Asiler, M., & Sahillioğlu, Y. (2024). 3D geometric kernel computation in polygon mesh structures. Computers & Graphics, 103951. <br />
 
© 2024 Published by Elsevier Ltd. <br /> <br />


**>>>> DISTRIBUTION** <br /> <br />
Windows 10 or newer.


**>>>> DEPENDENCIES**
- CGAL 5.3 or newer.
- Eigen 3.3.7 or newer.
- Boost 1.76.0 or newer.
- Visual Studio 2019 or newer.

You can use C++ package management tools such as [vcpkg](https://vcpkg.io/en/) and [nuget](https://www.nuget.org/) to quickly install dependencies.

If you already have vcpkg installed and the path of your vcpkg.exe is ```C:\vcpkg\vcpkg.exe```, then you can run ```install-3rd-parties.bat``` directly, otherwise you can install the dependency manually by following the steps below:

+ Open powershell or cmd in the vcpkg folder and enter the following command to integrate vcpkg into all visual studio projects:

	```
	vcpkg integrate install
 	```

+ Install 3rd parties:

	```
	vcpkg install boost:x64-windows   
	vcpkg install cgal:x64-windows    
	vcpkg install eigen3:x64-windows  
	```
 

**>>>> RUN** <br /> <br />
If your visual studio 2019 (Community) uses the default installation path, which is C:\Program Files\Microsoft Visual Studio\2019\Community\, you can run clean&build&run-sample.bat directly, and the output kernel mesh file (.off) for an input star-shaped mesh will be generated in the folder that you specify in the console after the program runs.


**>>>> USAGE** <br /> <br />
(The .exe is given in the EXE folder.)  <br />
When you execute the code, the following command-line questions come up: <br /> <br />
**1.** "Enter the command type:" <br />
--> It should be either one of the followings:
| File | Task |
| --- | --- |
| approx_ker                 | Single kernel computation by the proposed approximation algorithm          |
| kernel_by_cgal             | Single kernel computation by CGAL                                          |
| batch_approx_ker           | Batch kernel computation by proposed approximation algorithm               |
| batch_kernel_by_cgal       | Batch kernel computation by CGAL                                           |
| compare_algos              | Compare the CGAL's and the proposed approximation algorithm's outputs.     |

**2.** "Enter the input path (.off/.obj):"   or    "Enter the input folder:" <br />
--> Give the complete path using slash "/". (NOT backslash "\")
   
**3.** "Enter the output path:"   or   "Enter the output folder:  <br />
--> It should be either one of the followings:
- The complete path of the output(s)
- d
- D
  
--> "d" of "D" refers to the "default". <br />
--> The output(s) is written into the path where the input(s) is specified. <br />
--> The output(s) is .off files.
	
<br />  

If the Kernel Approximation Algorithm is called, parameter settings are needed:

**4.** "Enter the number of rays [default -> 30] :" <br />
--> It should be either one of the followings:
- an integer
- d
- D
  
--> "d" of "D" refers to the "default".

**5.** "Enter the recursion depth limit [default -> 3] :" <br />
--> It should be either one of the followings:
- an integer
- d
- D
  
--> "d" of "D" refers to the "default".

**6.** "Enter the ray distribution type [default -> geometry-based] :" <br />
--> It should be either one of the followings:
- spherical-based
- geometry-based
- d
- D
  
--> "d" of "D" refers to the "default".

<br />

**>>>> CODE STRUCTURE** <br /> <br />
**1. Management**
| File | Task |
| --- | --- |
| Main.cpp                | Manages the command specifications.                                                                                                     |
| KernelComputation.cpp   | Manages the single/batch run of algorithms.                                                                                             |
| KernelExpansion.cpp     | Manages the general constructions before running any kernel computation algorithm. (Abstract class for kernel computation algorithms).  |
| KernelApproximation.cpp | Manages the operations of the proposed kernel approximation algorithm. (Derived class of KernelExpansion).                              |
| KernelByCGAL.cpp        | Manages the operations of CGAL's kernel computation algorithm. (Derived class of KernelExpansion).                                      |
| Parameters.cpp          | Manages the command-line specifications for paramaters used in the proposed kernel approximation algorithm.                             |
| sdlp.cpp                | Manages the operations of single kernel point finder method. (Taken from [here](https://github.com/ZJU-FAST-Lab/SDLP) . Copyright (c) 1990 Michael E. Hohmeyer & 2021 Zhepei Wang.) |

**2. Models**
| File | Task |
| --- | --- |
| Mesh.cpp                   | Defines the triangular mesh structure and its primary operations.                                                        |
| MeshTools.cpp              | Defines secondary operations regarding mesh structures.                                                                  |
| BasicMeshElements.cpp      | Defines the basic mesh elements (and its operations) including Vertex, Edge, Triangle classes.                           |
| BasicGeometricElements.cpp | Defines the basic geometric structures (and its operations) including Line, HalfPlane, Plane, Halfspace.                 |

**3. Utils**
| File | Task |
| --- | --- |
| BaseGeoOpUtils.cpp   | Defines general geometric operations such as line-plane intersections, point-plane distance calculations, etc.                            |
| BaseMathOpUtils.cpp  | Defines general arithmetical operations such as cross product of vectors, vector length measurements, etc.                                |
| CGALUtils.cpp        | Defines the manager functions for some algorithms provided by CGAL such as convex hull computation, Hausdorff distance calculations, etc. |
| CommonUtils.cpp      | Defines general-usage functions such as string splitting, etc.

<br />

**>>>> CONTACT** <br /> <br />
If you have any problem about this code, feel free to contact asiler@ceng.metu.edu.tr .
