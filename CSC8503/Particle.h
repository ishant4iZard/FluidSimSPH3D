#pragma once
#include <Vector3.h>

namespace NCL {
	namespace CSC8503 {
		using namespace Maths;
		class Particle {
		public:
			Vector3 Position				= Vector3();
			Vector3 PredictedPosition		= Vector3();
			Vector3 Velocity				= Vector3();
			Vector3 PressureAcceleration	= Vector3();
			Vector3 Force					= Vector3();
			Vector3 Acceleration			= Vector3();

			float density	= 0;
			float pressure	= 0;

			Vector3 GridPos = Vector3();
			int Gridhash;

		};

	}
}