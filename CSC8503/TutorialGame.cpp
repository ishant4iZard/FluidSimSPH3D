#include "TutorialGame.h"
#include "GameWorld.h"
#include "RenderObject.h"
#include "TextureLoader.h"

#include <Maths.h>


using namespace NCL;
using namespace CSC8503;



TutorialGame::TutorialGame() : controller(*Window::GetWindow()->GetKeyboard(), *Window::GetWindow()->GetMouse()) {
	world		= new GameWorld();
#ifdef USEVULKAN
	renderer	= new GameTechVulkanRenderer(*world);
	renderer->Init();
	renderer->InitStructures();
#else 
	renderer = new GameTechRenderer(*world);
#endif

	useGravity		= true;
	inSelectionMode = false;

	world->GetMainCamera().SetController(controller);

	controller.MapAxis(0, "Sidestep");
	controller.MapAxis(1, "UpDown");
	controller.MapAxis(2, "Forward");

	controller.MapAxis(3, "XLook");
	controller.MapAxis(4, "YLook");

	positionList = new Vector3[9]
	{	
		Vector3(0, 0, 0),
		Vector3(0, 0, 1),
		Vector3(0, 0, -1),
		Vector3(0, 1, 0),
		Vector3(0, 1, 1),
		Vector3(0, 1, -1),
		Vector3(0, -1, 0),
		Vector3(0, -1, 1),
		Vector3(0, -1, -1)
	};

	InitialiseAssets();

	InitDefaultFloor();
}

/*

Each of the little demo scenarios used in the game uses the same 2 meshes, 
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	sphereMesh	= renderer->LoadMesh("sphere.msh");

	basicTex	= renderer->LoadTexture("checkerboard.png");
	sandTex		= renderer->LoadTexture("sand.jpg");
	basicShader = renderer->LoadShader("scene.vert", "scene.frag");
	instancedParticleShader = renderer->LoadShader("sceneInstanced.vert", "scene.frag");

	InitCamera();
}

TutorialGame::~TutorialGame()	{
	delete sphereMesh;

	delete basicTex;
	delete basicShader;
	delete instancedParticleShader;
	delete renderer;
	delete world;
}

void TutorialGame::UpdateGame(float dt) {

	if (!inSelectionMode) {
		world->GetMainCamera().UpdateCamera(dt);
	}

	world->UpdateWorld(dt);

	positionList[2] = positionList[2] + Vector3(0, 0, 1) * dt;

	ParticleObject->GetRenderObject()->GetMesh()->UpdateParticlesPositionInstance(positionList, 9);

	renderer->Update(dt);

	renderer->Render();

	Debug::UpdateRenderables(dt);
}

void TutorialGame::InitCamera() {
	world->GetMainCamera().SetNearPlane(0.1f);
	world->GetMainCamera().SetFarPlane(500.0f);
	world->GetMainCamera().SetPitch(-15.0f);
	world->GetMainCamera().SetYaw(315.0f);
	world->GetMainCamera().SetPosition(Vector3(-60, 40, 60));
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple' 
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass , bool isHollow , float elasticity) {
	GameObject* sphere = new GameObject("sphere");

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, instancedParticleShader));

	world->AddGameObject(sphere);

	sphereMesh->SetInstanceModelMatrices(positionList, 9);

	return sphere;
}

void TutorialGame::InitDefaultFloor() {
	ParticleObject = AddSphereToWorld(Vector3(-64, -20, 0), 0.5f);
}


