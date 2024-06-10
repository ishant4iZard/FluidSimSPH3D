#include "../NCLCoreClasses/KeyboardMouseController.h"

#pragma once
#include "GameTechRenderer.h"
#ifdef USEVULKAN
#include "GameTechVulkanRenderer.h"
#endif


namespace NCL {
	namespace CSC8503 {
		class SPH;
		class TutorialGame		
		{
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);


		protected:
			void InitialiseAssets();

			void InitComputeShaders();

			GLuint CompileComputeShader(const std::string& filename);

			void InitCamera();

			GameObject* AddSphereToWorld(const Vector3& position, float radius, float inverseMass = 10.0f , bool isHollow = false, float elasticity = 0.81f);


#ifdef USEVULKAN
			GameTechVulkanRenderer*	renderer;
#else
			GameTechRenderer* renderer;
#endif
			GameWorld*			world;

			KeyboardMouseController controller;

			bool useGravity;
			bool inSelectionMode;

			Mesh*	sphereMesh	= nullptr;

			Texture*	basicTex	= nullptr;
			Texture*	sandTex		= nullptr;
			Shader*		basicShader = nullptr;
			Shader*		instancedParticleShader = nullptr;

			GLuint setParticlesInGridsSource;
			GLuint parallelSortSource;
			GLuint updateDensityPressureSource;
			GLuint updatePressureAccelerationSource;
			GLuint updateParticlesSource;
			
			void InitDefaultFloor();
			Vector3* positionList;

			GameObject* ParticleObject;

			SPH* water;
			unsigned int numParticles;


			
		private:

		};
	}
}

