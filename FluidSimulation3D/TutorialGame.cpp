#include "Assets.h"
#include "GameWorld.h"
#include "ParticlePhysics.h"
#include "RenderObject.h"
#include "TextureLoader.h"
#include "TutorialGame.h"
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
	
	water = new SPH(*world);

	InitialiseAssets();

	InitParticle();

	m_particleBuffer = water->getparticleBuffer();
	renderer->setMarchingCubesBuffer(water->getTriangleBuffer());

	(static_cast<OGLShader*>(ParticleObject->GetRenderObject()->GetShader()))->setParticleBuffer(m_particleBuffer);
	
}


/*

Each of the little demo scenarios used in the game uses the same 2 meshes, 
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	sphereMesh	= renderer->LoadMesh("Cube.msh");

	basicTex	= renderer->LoadTexture("checkerboard.png");
	sandTex		= renderer->LoadTexture("sand.jpg");
	basicShader = renderer->LoadShader("scene.vert", "scene.frag");
	instancedParticleShader = renderer->LoadShader("sceneInstanced.vert", "scene.frag");

	InitComputeShaders();

	InitCamera();
}

void NCL::CSC8503::TutorialGame::InitComputeShaders()
{
	/*setParticlesInGridsSource =			CompileComputeShader("setParticlesInGrids.comp");
	parallelSortSource =				CompileComputeShader("parallelSort.comp");
	updateDensityPressureSource =		CompileComputeShader("updateDensityPressure.comp");
	updatePressureAccelerationSource =	CompileComputeShader("updatePressureAcceleration.comp");
	updateParticlesSource =				CompileComputeShader("updateParticles.comp");*/

	water->setParticlesInGridsSource= CompileComputeShader("setParticlesInGrids.comp"); 
	water->parallelSortSource = CompileComputeShader("SortParticles.comp");
	water->HashTableSource = CompileComputeShader("HashLookupTable.comp");
	water->updateDensityPressureSource = CompileComputeShader("calcDensityandPressure.comp");
	water->updatePressureAccelerationSource = CompileComputeShader("calcPressureForce.comp");
	water->updateParticlesSource = CompileComputeShader("UpdateParticles.comp");
	water->resetHashTableSource = CompileComputeShader("ResetHashLookupTable.comp");
	water->preMarchingCubesSource = CompileComputeShader("preMarchingCubes.comp");
	water->MarchingCubesSource = CompileComputeShader("MarchingCubes.comp");

}

GLuint TutorialGame::CompileComputeShader(const std::string& filename)
{
	GLuint programID = glCreateProgram();
	std::string fileContents = "";
	
	if (Assets::ReadTextFile(Assets::SHADERDIR + filename, fileContents)) {
		OGLShader::Preprocessor(fileContents);

		GLuint shaderIDs = glCreateShader(GL_COMPUTE_SHADER);

		std::cout << "Reading " << " shader " << filename << "\n";

		const char* stringData = fileContents.c_str();
		int			stringLength = (int)fileContents.length();
		glShaderSource(shaderIDs, 1, &stringData, nullptr);
		glCompileShader(shaderIDs);

		GLint success;
		glGetShaderiv(shaderIDs, GL_COMPILE_STATUS, &success);

		if (success == GL_TRUE) {
			glAttachShader(programID, shaderIDs);
		}
		else {
			std::cout << " shader " << filename << " has failed!" << "\n";
		}

		glDeleteShader(shaderIDs);

	}
	
	GLint success;

	glLinkProgram(programID);
	glGetProgramiv(programID, GL_LINK_STATUS, &success);

	if (!success) {
		GLint maxLength = 0;
		glGetProgramiv(programID, GL_INFO_LOG_LENGTH, &maxLength);
		std::vector<GLchar> errorLog(maxLength);
		glGetProgramInfoLog(programID, maxLength, &maxLength, &errorLog[0]);
		std::cerr << "Error linking program:" << std::endl << errorLog.data() << std::endl;
		glDeleteProgram(programID);
		return 0;
	}
	else
	{
		std::cout << " shader " << " Loaded!" << "\n";
	}


	return programID;
}

TutorialGame::~TutorialGame()	{
	delete sphereMesh;

	delete basicTex;
	delete basicShader;
	delete instancedParticleShader;
	delete renderer;
	delete world;

	delete water;
}

void TutorialGame::UpdateGame(float dt) {

	if (!inSelectionMode) {
		world->GetMainCamera().UpdateCamera(dt);
	}

	world->UpdateWorld(dt);

	handleInput();
	updateUI();

	water->Update(dt);

	renderer->Update(dt);
	renderer->Render();
	Debug::UpdateRenderables(dt);

}

void TutorialGame::InitCamera() {
	world->GetMainCamera().SetNearPlane(0.1f);
	world->GetMainCamera().SetFarPlane(1000.0f);
	world->GetMainCamera().SetPitch(-10.0f);
	world->GetMainCamera().SetYaw(-90.0f);
	world->GetMainCamera().SetPosition(Vector3(-350, 150, 150));
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple' 
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass , bool isHollow , float elasticity) {
	GameObject* sphere = new GameObject("sphere");

	Vector3 sphereSize = Vector3(radius, radius, radius);
	/*SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);*/

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, instancedParticleShader));

	sphere->GetRenderObject()->SetColour(Vector4(0, 0, 1, 0.9f));

	world->AddGameObject(sphere);

	sphereMesh->SetInstanceCount(water->getNumParticles());

	return sphere;
}

void NCL::CSC8503::TutorialGame::handleInput()
{
	if (Window::GetKeyboard()->KeyDown(KeyCodes::F1)) {
		renderer->setRenderParticles(false);
		renderer->setRenderSurface(true);
		water->setRenderParticles(false);
		water->setRenderSurface(true);
	}
	if (Window::GetKeyboard()->KeyDown(KeyCodes::F2)) {
		renderer->setRenderParticles(true);
		renderer->setRenderSurface(false);
		water->setRenderParticles(true);
		water->setRenderSurface(false);
	}
	if (Window::GetKeyboard()->KeyDown(KeyCodes::F3)) {
		renderer->setRenderParticles(true);
		renderer->setRenderSurface(true);
		water->setRenderParticles(true);
		water->setRenderSurface(true);
	}
}

void NCL::CSC8503::TutorialGame::updateUI()
{
	Debug::Print("Press F1 to render surface", Vector2(5, 85), Vector4(1, 0, 0, 1));
	Debug::Print("Press F2 to render particles", Vector2(5, 90), Vector4(1, 0, 0, 1));
	Debug::Print("Press F3 to render both", Vector2(5, 95), Vector4(1, 0, 0, 1));
}

void TutorialGame::InitParticle() {
	ParticleObject = AddSphereToWorld(Vector3(0, 0, 0), 0.5f);
}


