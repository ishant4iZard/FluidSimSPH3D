#pragma once
#include "Particle.h"
#include <execution>

#define PI 3.14159f

struct boundingArea {
	int left = 20;
	int right = 1260;
	int bottom = 700;
	int top = 20;
};

namespace NCL {
	namespace CSC8503 {
		using namespace Maths;
		class SPH
		{
		private:

#pragma region particles
			Particle* particles;

			int numParticles;
			float particleRadius;
			float smoothingRadius;
			float particleSpacing;
			float mass = 1.0f;
			float dampingRate = 0.98f;

#pragma endregion

			int HorGrids;
			int VerGrids;
			int* hashLookupTable;

			boundingArea fence;

			bool gravityEnabled = 1;
			Vector3 gravity = Vector3(0.f, 9.8f, 0.f);

			double targetDensity = 20.f;
			float pressureMultiplier = 0.001f;
			float viscosityMultiplier = 3.f;

			Vector3 offsetsGrids[27] = {
				Vector3(0,0,0),
				Vector3(0,0,1),
				Vector3(0,0,-1),
				Vector3(0,1,0),
				Vector3(0,1,1),
				Vector3(0,1,-1),
				Vector3(0,-1,0),
				Vector3(0,-1,1),
				Vector3(0,-1,-1),
				Vector3(1,0,0),
				Vector3(1,0,1),
				Vector3(1,0,-1),
				Vector3(1,1,0),
				Vector3(1,1,1),
				Vector3(1,1,-1),
				Vector3(1,-1,0),
				Vector3(1,-1,1),
				Vector3(1,-1,-1),
				Vector3(-1,0,0),
				Vector3(-1,0,1),
				Vector3(-1,0,-1),
				Vector3(-1,1,0),
				Vector3(-1,1,1),
				Vector3(-1,1,-1),
				Vector3(-1,-1,0),
				Vector3(-1,-1,1),
				Vector3(-1,-1,-1)
			};

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

			float vectorMagnitude(Vector3 vector2) {
				float r = sqrtf((vector2.x * vector2.x) + (vector2.y * vector2.y));
				return r;
			}

			Vector3 GetRandomDir() {
				float x = (rand() % 100) / 100.0f;
				float y = (rand() % 100) / 100.0f;
				float z = (rand() % 100) / 100.0f;

				Vector3 a(x, y , z);
				return a.Normalised();
			}

			int cellHash(int x, int y) {
				return y * HorGrids + x;
			}

			void resetHashLookupTable() {
				std::fill(std::execution::par, hashLookupTable, hashLookupTable + HorGrids * VerGrids, INT_MAX);
			}
#pragma endregion


			void randomPositionStart(float screenWidth, float screeenHeight);
			void GridStart(float screenWidth, float screeenHeight);


			double calcDensityGrid(int particleIndex, Vector3 gridPos);
			Vector3 calcPressureForceGrid(int particleIndex, Vector3 gridPos);


			void SetParticlesInGridsHashing();
			void UpdateDensityandPressureGrid();
			void UpdatePressureAccelerationGrid();
			void updateParticle(float dt);

		public:

			SPH(int inNumParticles, float screenWidth, float screeenHeight);
			~SPH();

			void Update(float dt);
		};
	}
}