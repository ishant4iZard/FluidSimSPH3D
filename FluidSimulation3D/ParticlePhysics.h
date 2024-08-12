#pragma once
#include "GameTechRenderer.h"
#include "Particle.h"
#include <execution>

#define PI 3.14159f

struct boundingArea {
	int left = 0;
	int right = 250;
	int bottom = 400;
	int top = 0;
	int front = 0;
	int back = 350;
};

namespace NCL {
	namespace CSC8503 {
		using namespace Maths;

		class GameWorld;
		class SPH
		{
		private:

#pragma region particles
			std::vector<Particle> particles;

			int numParticles;
			float particleRadius;
			float smoothingRadius;
			float particleSpacing;
			float mass = 1.0f;
			float dampingRate = 0.98f;

#pragma endregion
			std::vector<int> hashLookupTable;

			boundingArea fence;
			std::vector<std::pair<Vector3, Vector3>> fenceEdges;

			bool gravityEnabled = 1;
			Vector3 gravity = Vector3(0.f, -9.8f, 0.f);

			double targetDensity = 50.f;
			float pressureMultiplier = 1.f;
			float viscosityMultiplier = 0.7f;

			const unsigned int hashX = 15823;
			const unsigned int hashY = 9737333;
			const unsigned int hashZ = 440817757;

			int l2numparticles;
			int numStages;

			int sortingLocalSizeX;

			Vector3i gridSizeVec;

#pragma region HelperFunctionsAndConstants
			double SmoothingKernelMultiplier;
			double SmoothingKernelDerivativeMultiplier;

			double smoothingKernel(float inradius, float dst) {
				if (dst >= inradius) return 0;
				return pow(((inradius - dst) / 100.0f), 2) * SmoothingKernelMultiplier;
			}

			double smoothingKernerDerivative(float inradius, float dst) {
				if (dst >= inradius)return 0;
				return ((dst - inradius) / 100.0f) * SmoothingKernelDerivativeMultiplier;
			}

			double ConvertDensityToPressure(double density) {
				double deltaDensity = density - targetDensity;
				double m_pressure = deltaDensity * pressureMultiplier;
				return m_pressure;
			}

			Vector3 GetRandomDir() {
				float x = (rand() % 100) / 100.0f;
				float y = (rand() % 100) / 100.0f;
				float z = (rand() % 100) / 100.0f;

				Vector3 a(x, y , z);
				return a.Normalised();
			}

			int cellHash(int x, int y, int z) {
				return ((long)z * hashZ + (long)y * hashY + (long)x * hashX) % numParticles;
			}

			void resetHashLookupTable() {
				std::fill(std::execution::par, hashLookupTable.begin(), hashLookupTable.end(), INT_MAX);
			}

			int NextPowerOfTwo(int n) {
				return pow(2, ceil(log2(n)));
			}

			std::vector<std::pair<Vector3, Vector3>> calculateEdges(const boundingArea& bounds) {
				std::vector<std::pair<Vector3, Vector3>> edges;

				Vector3 vertices[8] = {
					{float(bounds.left), float(bounds.bottom), float(bounds.front)},
					{float(bounds.right), float(bounds.bottom), float(bounds.front)},
					{float(bounds.right), float(bounds.top), float(bounds.front)},
					{float(bounds.left), float(bounds.top), float(bounds.front)},
					{float(bounds.left), float(bounds.bottom), float(bounds.back)},
					{float(bounds.right), float(bounds.bottom), float(bounds.back)},
					{float(bounds.right), float(bounds.top), float(bounds.back)},
					{float(bounds.left), float(bounds.top), float(bounds.back)}
				};

				// Define the 12 edges of the bounding box
				edges.push_back({ vertices[0], vertices[1] });
				edges.push_back({ vertices[1], vertices[2] });
				edges.push_back({ vertices[2], vertices[3] });
				edges.push_back({ vertices[3], vertices[0] });

				edges.push_back({ vertices[4], vertices[5] });
				edges.push_back({ vertices[5], vertices[6] });
				edges.push_back({ vertices[6], vertices[7] });
				edges.push_back({ vertices[7], vertices[4] });

				edges.push_back({ vertices[0], vertices[4] });
				edges.push_back({ vertices[1], vertices[5] });
				edges.push_back({ vertices[2], vertices[6] });
				edges.push_back({ vertices[3], vertices[7] });

				return edges;
			}


#pragma endregion

			void GridStart();

			double calcDensityGrid(int particleIndex, Vector3i gridPos);
			Vector3 calcPressureForceGrid(int particleIndex, Vector3i gridPos);

			void SetParticlesInGridsHashing();
			void UpdateDensityandPressureGrid();
			void UpdatePressureAccelerationGrid();
			void updateParticle(float dt);

			void SetParticlesInGridsHashingGPU();
			void UpdateDensityandPressureGridGPU();
			void UpdatePressureAccelerationGridGPU();
			void updateParticleGPU(float dt);
			void resetHashLookupTableGPU();

			void PreMarchingCubes();
			void MarchingCubes();

			GLuint particleBuffer;
			GLuint hashLookupBuffer;
			GLuint postitionBuffer;
			GLuint CounterBuffer;
			GLuint TriangleBuffer;
			GLuint NeighbourParticlesBuffer;

			GLuint edgeTableBuffer;
			GLuint triTableBuffer;
			GLuint maxYbuffer;

			int local_size_x;

			int nextHashingFrame = 0;
			const int hashEveryNFrame = 4;

			GameWorld& gameWorld;
			float marchingCubesSize;
			int marchingCubesNoGrids;
			int marchingCubesIsoLevel;

			int numTriMarchingCubes;

			int numCubesXaxisMarchingCubes;
			int numCubesYaxisMarchingCubes;
			int numCubesZaxisMarchingCubes;

			int maxYparticle;

			//std::vector<Vector4> triangleData;
			//std::vector<float> NeigbourParticles;

			Matrix4 modelMatrix = Matrix4();
			Matrix4 modelViewMatrix;

			bool isRenderParticles;
			bool isRenderSurface;

		public:

			SPH(int inNumParticles, GameWorld& ingameWorld);
			~SPH();

			GLuint setParticlesInGridsSource;
			GLuint parallelSortSource;
			GLuint HashTableSource;
			GLuint updateDensityPressureSource;
			GLuint updatePressureAccelerationSource;
			GLuint updateParticlesSource;
			GLuint resetHashTableSource;
			GLuint preMarchingCubesSource;
			GLuint MarchingCubesSource;


			void Update(float dt);

			GLuint getparticleBuffer() {
				return particleBuffer;
			}

			GLuint getTriangleBuffer() {
				return TriangleBuffer;
			}

			void setRenderParticles(bool inIsRenderParticles) {
				isRenderParticles = inIsRenderParticles;
			}

			void setRenderSurface(bool inIsRenderSurface) {
				isRenderSurface = inIsRenderSurface;
			}
		};
	}
}