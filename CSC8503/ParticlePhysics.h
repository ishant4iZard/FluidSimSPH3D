#pragma once
#include "Particle.h"
#include <execution>

#define PI 3.14159f

struct boundingArea {
	int left = 0;
	int right = 120;
	int bottom = 450;
	int top = 0;
	int front = 0;
	int back = 250;
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

			int NoGridsX;
			int NoGridsY;
			int NoGridsZ;
			int* hashLookupTable;

			boundingArea fence;

			bool gravityEnabled = 1;
			Vector3 gravity = Vector3(0.f, -9.8f, 0.f);

			double targetDensity = 100.f;
			float pressureMultiplier = 0.001f;
			float viscosityMultiplier = 3.f;

			const unsigned int hashX = 15823;
			const unsigned int hashY = 9737333;
			const unsigned int hashZ = 440817757;


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
				std::fill(std::execution::par, hashLookupTable, hashLookupTable + numParticles, INT_MAX);
			}
#pragma endregion

			void randomPositionStart();
			void GridStart(Vector3* PosList);


			double calcDensityGrid(int particleIndex, Vector3 gridPos);
			Vector3 calcPressureForceGrid(int particleIndex, Vector3 gridPos);


			void SetParticlesInGridsHashing();
			void UpdateDensityandPressureGrid();
			void UpdatePressureAccelerationGrid();
			void updateParticle(float dt, Vector3* PosList);

		public:

			SPH(int inNumParticles, Vector3* PosList);
			~SPH();

			void Update(float dt, Vector3* PosList);
		};
	}
}