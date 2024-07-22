# FluidSimulation3D

FluidSimulation3D is a 3D fluid simulation project that implements a fluid simulation using the **Smoothed Particle Hydrodynamics (SPH)** method, integrated with the **Marching Cubes** algorithm for surface reconstruction. This project leverages both CPU and GPU computations, utilizing OpenGL compute shaders for efficient handling of particle updates, density and pressure calculations, and mesh generation.

The framework for this project was provided by Professor Richard Davison, and changes have been made according to specific requirements.

## Features

- **SPH Simulation**: Models fluid dynamics using the Smoothed Particle Hydrodynamics method.
- **Marching Cubes**: Generates a mesh representation of the fluid surface for visualization.
- **Morton Order Spatial Indexing**: Uses Morton order (Z-order curve) for efficient neighbor searches and particle updates.
- **Boundary Conditions**: Ensures particles remain within the simulation volume.
- **GPU Acceleration**: Utilizes OpenGL compute shaders for enhanced performance in particle interactions and mesh generation.
- **Parallel Processing**: Improves computational efficiency by leveraging multi-threading.

## Key Components

- **SPH Class**: Initializes simulation parameters, including particle count and smoothing radius. Sets up OpenGL buffers for storing particle data and neighbor information.
- **Particle Updates**: Handles updates to particle positions, velocities, and interactions using both CPU and GPU resources.
- **Density and Pressure Calculations**: Computes fluid density and pressure to simulate realistic fluid behavior.
- **Marching Cubes Integration**: Constructs a 3D mesh of the fluid surface for visualization.

## Project Structure

The project is structured into several key components:

- **NCLCoreClasses**: Core classes for mathematical and utility functions.
- **CSC8503CoreClasses**: Additional core classes, related to the CSC8503 course.
- **OpenGLRendering**: Contains rendering logic and OpenGL setup.
- **FluidSimulation3D**: The main simulation executable.

### Source Files

- **Header Files**:
  - `Particle.h`
  - `ParticlePhysics.h`
  - `MarchingCubesConstants.h`
  - `GameTechRenderer.h`
  - `TutorialGame.h`

- **Source Files**:
  - `ParticlePhysics.cpp`
  - `GameTechRenderer.cpp`
  - `Main.cpp`
  - `TutorialGame.cpp`

### Shader Files

Shaders are located in the `Assets/Shaders` directory. The directory contains several shaders, including:
- Compute shaders
- `marchingCubesRendering.frag` (fragment shader) (custom)
- `marchingCubesRendering.vert` (vertex shader) (custom)

The custom shaders are specifically developed for fluid simulation, rendering, and the Marching Cubes algorithm. Additional shaders are also included in this directory for various other purposes within the simulation.

## Requirements

- **CMake**: Version 3.16.0 or higher
- **Visual Studio**: Required for building on Windows, with support for x64 architecture
- **C++**: C++20 standard

## Getting Started

To build the project, follow these steps:

1. **Clone the Repository**:
    ```bash
    git clone https://github.com/your-repo/FluidSimulation3D.git
    cd FluidSimulation3D
    ```

2. **Create a Build Directory**:
    ```bash
    mkdir build
    cd build
    ```

3. **Configure the Project**:
    ```bash
    cmake ..
    ```

4. **Build the Project**:
    ```bash
    cmake --build .
    ```

## Configuration

### CMake Settings

- **CMake Version**: 3.16.0 or higher
- **C++ Standard**: C++20
- **Architecture**: x64 (Windows)
- **Configuration Types**: Debug, Release

### Source Groups

The source files are organized into header and source groups for better management in IDEs like Visual Studio.

### Compile and Link Options

- **MSVC Options**:
  - Optimization and warning levels are set based on configuration (Debug or Release).
  - Precompiled headers are used for faster compilation.
  - Linker options include optimizations for Release configuration.

### Dependencies

The project links against:
- **Winmm.lib** (Windows Multimedia API)
- **NCLCoreClasses**: Core utility library
- **CSC8503CoreClasses**: Additional core classes
- **OpenGLRendering**: OpenGL rendering library

## Additional Information

- **Solution Folders**: Enabled for better organization in Visual Studio.
- **Startup Project**: `FluidSimulation3D` is set as the default startup project.
- **Asset Root**: The `Assets/` directory is used for storing assets, including shader files.

## Contributing

Contributions are welcome! Please fork the repository and submit pull requests for any improvements or bug fixes.

## License

### Original Codebase

This project includes code from the framework provided by Professor Richard Davison. The original codebase is licensed under Richard Davison. Please refer to the [Original License](LICENSE_Professor) file for the complete license terms.

### My Contributions

The new code contributed by Ishant Agarwalla is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

For any questions or support, please contact ishant0203@gmail.com(mailto:ishant0203@gmail.com).
