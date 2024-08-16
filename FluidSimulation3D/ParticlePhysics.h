#pragma once
#include "GameTechRenderer.h"
#include "Particle.h"
#include <execution>

#define PI 3.14159f

struct boundingArea {
	int left = 0;
	int right = 250;
	int bottom = 0;
	int top = 400;
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

			std::vector<Particle> m_particles;
			std::vector<int> m_hashLookupTable;

#pragma region particleSettings
			unsigned int	m_numParticles;
			float			m_particleRadius;
			float			m_particleSmoothingRadius;
			float			m_particleSpacing;
			float			m_particleMass;
			float			m_particleDampingRate;
			bool			m_isParticleGravityEnabled;
			Vector3			m_particleGravity;

			double			m_particleTargetDensity;
			float			m_particlePressureMultiplier;
			float			m_particleViscosityMultiplier;
			double			m_particleSmoothingKernelMultiplier;
			double			m_particleSmoothingKernelDerivativeMultiplier;

			const int		kHashFrameInterval = 4;
#pragma endregion

			int				nextHashingFrame = 0;
			boundingArea	m_fence;
			std::vector<std::pair<Vector3, Vector3>> m_fenceEdges;

#pragma region HelperFunctionsAndVariables
			int l2numparticles;
			int numStages;

			int m_sortingLocalSizeX;
			int m_localSizeX;

			const unsigned int kHashX = 15823;
			const unsigned int kHashY = 9737333;
			const unsigned int kHashZ = 440817757;

			inline double smoothingKernel(float inradius, float dst) {
				if (dst >= inradius) return 0;
				return pow(((inradius - dst) / 100.0f), 2) * m_particleSmoothingKernelMultiplier;
			}

			inline double smoothingKernerDerivative(float inradius, float dst) {
				if (dst >= inradius)return 0;
				return ((dst - inradius) / 100.0f) * m_particleSmoothingKernelDerivativeMultiplier;
			}

			inline double ConvertDensityToPressure(double density) {
				double deltaDensity = density - m_particleTargetDensity;
				double m_pressure = deltaDensity * m_particlePressureMultiplier;
				return m_pressure;
			}

			inline Vector3 GetRandomDir() {
				float x = (rand() % 100) / 100.0f;
				float y = (rand() % 100) / 100.0f;
				float z = (rand() % 100) / 100.0f;

				Vector3 a(x, y , z);
				return a.Normalised();
			}

			inline int cellHash(int x, int y, int z) {
				return ((long)z * kHashZ + (long)y * kHashY + (long)x * kHashX) % m_numParticles;
			}

			inline void resetHashLookupTable() {
				std::fill(std::execution::par, m_hashLookupTable.begin(), m_hashLookupTable.end(), INT_MAX);
			}

			inline int NextPowerOfTwo(int n) {
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

			// CPU-Based Functions (currently redacted)
#pragma region CPUFunction
//			double calcDensityGrid(int particleIndex, Vector3i gridPos);
//			Vector3 calcPressureForceGrid(int particleIndex, Vector3i gridPos);
//			void SetParticlesInGridsHashing();
//			void UpdateDensityandPressureGrid();
//			void UpdatePressureAccelerationGrid();
//			void updateParticle(float dt);
#pragma endregion

			// GPU-Based Functions
#pragma region GPUFunction
			void SetParticlesInGridsHashingGPU();
			void UpdateDensityandPressureGridGPU();
			void UpdatePressureAccelerationGridGPU();
			void updateParticleGPU(float dt);
			void resetHashLookupTableGPU();

			void PreMarchingCubes();
			void MarchingCubes();
#pragma endregion

#pragma region Buffers
			GLuint m_particleBuffer;
			GLuint m_hashLookupBuffer;
			GLuint m_counterBuffer;
			GLuint m_triangleBuffer;
			GLuint m_neighbourParticlesBuffer;
			GLuint m_maxHeightBuffer;

			//marching cubes constant buffers
			GLuint m_edgeTableBuffer;
			GLuint m_triTableBuffer;
#pragma endregion

			GameWorld& gameWorld;

#pragma region marchingCubesVariables
			float m_marchingCubesSize;
			int m_marchingCubesNoGrids;
			int m_marchingCubesIsoLevel;

			int m_numTriMarchingCubes;

			int m_numCubesXaxisMarchingCubes;
			int m_numCubesYaxisMarchingCubes;
			int m_numCubesZaxisMarchingCubes;

			bool m_isRenderParticles;
			bool m_isRenderSurface;
#pragma endregion

			int m_maxParticleHeight;

		public:

			SPH(GameWorld& ingameWorld);
			~SPH();

			void Update(float dt);

			unsigned int getNumParticles(){ return m_numParticles; }

			GLuint getparticleBuffer() {
				return m_particleBuffer;
			}

			GLuint getTriangleBuffer() {
				return m_triangleBuffer;
			}

			void setRenderParticles(bool inIsRenderParticles) {
				m_isRenderParticles = inIsRenderParticles;
			}

			void setRenderSurface(bool inIsRenderSurface) {
				m_isRenderSurface = inIsRenderSurface;
			}

		private:
			void InitializeParticles();
			void InitializeMarchingCubesVariables();
			void InitializeHashingAndSortingVariables();
			void InitializeOpenGLBuffers();

		public:
			GLuint setParticlesInGridsSource;
			GLuint parallelSortSource;
			GLuint HashTableSource;
			GLuint updateDensityPressureSource;
			GLuint updatePressureAccelerationSource;
			GLuint updateParticlesSource;
			GLuint resetHashTableSource;
			GLuint preMarchingCubesSource;
			GLuint MarchingCubesSource;
		};
	}
}