We introduce a novel approach to compute the geometric kernel of a polygon mesh embedded in 3D.

** DEPENDENCIES **
- Coin3D
- CGAL
- Eigen
- Boost

**CODE STRUCTURE** <br />
**1. Management**
| File | Task |
| --- | --- |
| Main.cpp                | Manages the command specifications.                                                                                                     |
| KernelComputation.cpp   | Manages the single/batch run of algorithms.                                                                                             |
| KernelExpansion.cpp     | Manages the general constructions before running any kernel computation algorithm. (Abstract class for kernel computation algorithms).  |
| KernelApproximation.cpp | Manages the operations of the proposed kernel approximation algorithm. (Derived class of KernelExpansion).                              |
| KernelByCGAL.cpp        | Manages the operations of CGAL's kernel computation algorithm. (Derived class of KernelExpansion).                                      |
| Parameters.cpp          | Manages the command-line specifications for paramaters used in the proposed kernel approximation algorithm.                             |

**2. Models**
| Mesh.cpp                   | Defines the triangular mesh structure (and its primary operations).                                                        |
| MeshTools.cpp              | Secondary operations regarding mesh structures.                                                                       |
| BasicMeshElements.cpp      | Defines the basic mesh elements (and its operations) including Vertex, Edge, Triangle classes.                |
| BasicGeometricElements.cpp | Defines the basic geometric structures (and its operations) including Line, HalfPlane, Plane, Halfspace. |

**3. Utils**


**=== USAGE of .exe: ===**
**1.** "Enter the command type:" 

	      --> It should be either one of the followings:

            <approx_ker>							      : Single kernel computation by the proposed approximation algorithm
		        <kernel_by_cgal>					      : Single kernel computation by CGAL
		        <batch_approx_ker>					    : Batch kernel computation by proposed approximation algorithm
		        <batch_kernel_by_cgal>			    : Batch kernel computation by CGAL
		        <draw>							            : Draw the given mesh to the screen
		        <visual_comparison_of_algos>		: Compare the CGAL's and the proposed approximation algorithm's outputs.
										                          CGAL/the_proposed_algorithm is drawn to the left/right of the screen. 

**2.** "Enter the input path (.off/.obj):"   or    "Enter the input folder:"
	      --> Give the complete path using slash "/". (NOT backslash "\")
   
**3.** "Enter the output path:"   or   "Enter the output folder:
	      --> It should be either one of the followings:
		        <The complete path of the output(s)>, <d>, <D>
	          ("d" of "D" refers to the "default". The output(s) is wirtten into the path where the input(s) is specified.
	            The output(s) is .off files.)
	
**4.** "Draw to the screen? (y/n):"
        --> It should be either one of the followings:
            <y>, <Y>, <n>, <N>
		        ("y" or "Y" refers to "yes". Draws the output(s) to the screen.
	          "n" or "N" refers to "no". It does not draw anything to the screen.)

If the Kernel Approximation Algorithm is called, parameter settings are needed:

**5.** "Enter the number of rays [default -> 30] :"
        --> It should be either one of the followings:
            <an integer>, <d>, <D>

**6.** "Enter the recursion depth limit [default -> 3] :"
        --> It should be either one of the followings:
            <an integer>, <d>, <D>

**7.** "Enter the ray distribution type [default -> geometry-based] :"
        --> It should be either one of the followings:
            <spherical-based>, <geometry-based>, <d>, <D>
