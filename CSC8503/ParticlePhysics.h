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

			bool gravityEnabled = 1;
			Vector3 gravity = Vector3(0.f, -9.8f, 0.f);

			double targetDensity = 100.f;
			float pressureMultiplier = 0.001f;
			float viscosityMultiplier = 3.f;

			const unsigned int hashX = 15823;
			const unsigned int hashY = 9737333;
			const unsigned int hashZ = 440817757;


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

#pragma endregion

			void randomPositionStart();
			void GridStart(Vector3* PosList);


			double calcDensityGrid(int particleIndex, Vector3i gridPos);
			Vector3 calcPressureForceGrid(int particleIndex, Vector3i gridPos);


			void SetParticlesInGridsHashing();
			void UpdateDensityandPressureGrid();
			void UpdatePressureAccelerationGrid();
			void updateParticle(float dt, Vector3* PosList);

			void SetParticlesInGridsHashingGPU();
			void UpdateDensityandPressureGridGPU();
			void UpdatePressureAccelerationGridGPU();
			void updateParticleGPU(float dt, Vector3* PosList);
			void resetHashLookupTableGPU();

			void PreMarchingCubes();
			void MarchingCubes();


			GLuint particleBuffer;
			GLuint hashLookupBuffer;
			GLuint postitionBuffer;
			GLuint CounterBuffer;
			GLuint TriangleBuffer;
			GLuint NeighbourParticlesBuffer;

			int local_size_x;

			int nextHashingFrame = 0;
			const int hashEveryNFrame = 3;

			GameWorld& gameWorld;
			float marchingCubesSize;
			int marchingCubesNoGrids;
			int marchingCubesIsoLevel;

			int numTriMarchingCubes;

			int numCubesXaxisMarchingCubes;
			int numCubesYaxisMarchingCubes;
			int numCubesZaxisMarchingCubes;

			//std::vector<Vector4> triangleData;
			//std::vector<float> NeigbourParticles;

			Matrix4 modelMatrix = Matrix4();
			Matrix4 modelViewMatrix;

		public:

			SPH(int inNumParticles, Vector3* PosList, GameWorld& ingameWorld);
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


			void Update(float dt, Vector3* PosList);

			GLuint getparticleBuffer() {
				return particleBuffer;
			}

			GLuint getTriangleBuffer() {
				return TriangleBuffer;
			}
		};
	}
}