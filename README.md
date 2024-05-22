We introduce a novel approach to compute the geometric kernel of a polygon mesh embedded in 3D. <br />
This is the source code for our paper "3D Geometric Kernel Computation in Polygon Mesh Structures." <br />

**>>>> DISTRIBUTION** <br /> <br />
Windows 10 or newer.

<br />

**>>>> DEPENDENCIES**
- Coin3D.
- CGAL 5.3 or newer.
- Eigen 3.3.7 or newer.
- Boost 1.76.0 or newer.
- Visual Studio 2019 or newer.

You can use C++ package management tools such as vcpkg and nuget to quickly install dependencies.

You can install the dependency manually by following the steps below:

Open powershell or cmd in the vcpkg folder and enter the following command to integrate vcpkg into all visual studio projects:

	vcpkg integrate install
 
Boost installation:
 
	vcpkg install boost:x64-windows 

Eigen installation:

	Download the Eigen library: https://eigen.tuxfamily.org/index.php?title=Main_Page

CGAL installation:

	(Recommended to follow the manual for details: https://doc.cgal.org/latest/Manual/windows.html)
 	.\vcpkg.exe install yasm-tool:x86-windows
  	.\vcpkg.exe install cgal

Coin3D installation:

	Download the Coin3D library: https://github.com/coin3d/coin

Define the paths for Release mode, x86 platform: <br />
Visual Studio Project Properties:
+ C/C++ --> General (Additional include directories): 
	- Absolute path to eigen-3.3.7.
 	- Absolute path to CGAL/include.
  	- Absolute path to Coin3D/include.
  	- Absolute path to vcpkg/installed/x86-windows/include.

+ C/C++ --> Preprocessor (Preprocessor definitions):
	- Paste: WIN32;NDEBUG;_CONSOLE;COIN_DLL;SOWIN_DLL;

+ Linker --> General (Additional Library Directories):
  	- Absolute path to Coin3D/lib.
  	- Absolute path to vcpkg/installed/x86-windows/lib.

+ Linker --> Input (Additional dependencies):
	- Paste: coin2d.lib;sowin1d.lib;gmp.lib;mpfr.lib;
  
Add the following .dll files to your exe folder of the Visual Studio project.
- boost_filesystem-vc142-mt-x32-1_76.dll
- coin2d.dll
- gmp.dll
- mpfr.dll
- msvcr71d.dll
- sowin1d.dll
     
 
<br />

**>>>> USAGE** <br /> <br />
When you execute the code, the following command-line questions come up: <br /> <br />
**1.** "Enter the command type:" <br />
--> It should be either one of the followings:
| File | Task |
| --- | --- |
| approx_ker                 | Single kernel computation by the proposed approximation algorithm                                                                             |
| kernel_by_cgal             | Single kernel computation by CGAL                                                                                                             |
| batch_approx_ker           | Batch kernel computation by proposed approximation algorithm                                                                                  |
| batch_kernel_by_cgal       | Batch kernel computation by CGAL                                                                                                              |
| draw                       | Draw the given mesh to the screen                                                                                                             |
| visual_comparison_of_algos | Compare the CGAL's and the proposed approximation algorithm's outputs. CGAL/the_proposed_algorithm is drawn to the left/right of the screen.  |

**2.** "Enter the input path (.off/.obj):"   or    "Enter the input folder:" <br />
--> Give the complete path using slash "/". (NOT backslash "\")
   
**3.** "Enter the output path:"   or   "Enter the output folder:  <br />
--> It should be either one of the followings:
- The complete path of the output(s)
- d
- D
  
--> "d" of "D" refers to the "default". <br />
--> The output(s) is wirtten into the path where the input(s) is specified. <br />
--> The output(s) is .off files.
	
**4.** "Draw to the screen? (y/n):" <br />
--> It should be either one of the followings:
- y
- Y
- n
- N
  
--> "y" or "Y" refers to "yes". Draws the output(s) to the screen. <br />
--> "n" or "N" refers to "no". It does not draw anything to the screen.  
<br />  <br />

If the Kernel Approximation Algorithm is called, parameter settings are needed:

**5.** "Enter the number of rays [default -> 30] :" <br />
--> It should be either one of the followings:
- an integer
- d
- D
  
--> "d" of "D" refers to the "default".

**6.** "Enter the recursion depth limit [default -> 3] :" <br />
--> It should be either one of the followings:
- an integer
- d
- D
  
--> "d" of "D" refers to the "default".

**7.** "Enter the ray distribution type [default -> geometry-based] :" <br />
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
| sdlp.cpp                | Manages the operations of single kernel point finder method. (Taken from https://github.com/ZJU-FAST-Lab/SDLP . Copyright (c) 1990 Michael E. Hohmeyer & 2021 Zhepei Wang.) |

**2. Models**
| File | Task |
| --- | --- |
| Mesh.cpp                   | Defines the triangular mesh structure and its primary operations.                                                        |
| MeshTools.cpp              | Defines secondary operations regarding mesh structures.                                                                            |
| BasicMeshElements.cpp      | Defines the basic mesh elements (and its operations) including Vertex, Edge, Triangle classes.                             |
| BasicGeometricElements.cpp | Defines the basic geometric structures (and its operations) including Line, HalfPlane, Plane, Halfspace.                   |

**3. Utils**
| File | Task |
| --- | --- |
| BaseGeoOpUtils.cpp   | Defines general geometric operations such as line-plane intersections, point-plane distance calculations, etc.                            |
| BaseMathOpUtils.cpp  | Defines general arithmetical operations such as cross product of vectors, vector length measurements, etc.                                |
| CGALUtils.cpp        | Defines the manager functions for some algorithms provided by CGAL such as convex hull computation, Hausdorff distance calculations, etc. |
| CommonUtils.cpp      | Defines general-usage functions such as string splitting, etc.

**4. Views**
| File | Task |
| --- | --- |
| Painter.cpp      | Manages the drawing and painting of the shapes for the scene.                    |
| Scene.cpp        | Manages the scene construction variables such as camera, window properties, etc. |
| SceneManager.cpp | Manages the necessary function calls to setup the scene and draw its content.    |

<br />

**>>>> CONTACT** <br /> <br />
If you have any problem about this code, feel free to contact <br />
asiler@ceng.metu.edu.tr
