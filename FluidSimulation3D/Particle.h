#pragma once
#include <Vector3.h>

namespace NCL {
	namespace CSC8503 {
		using namespace Maths;
		class Particle {
		public:
			Vector3 Position				= Vector3();
			int padding1 = 0;
			Vector3 PredictedPosition		= Vector3();
			int padding2 = 0;
			Vector3 Velocity				= Vector3();
			int padding3 = 0;
			Vector3 PressureAcceleration	= Vector3();
			


			float density	= 0;
			float pressure	= 0;


			unsigned int Gridhash = INT_MAX;
			int padding7 = 0;
			int padding8 = 0;


		};

	}
}