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

	
	numParticles = 500000;
	
	water = new SPH(numParticles,*world);

	InitialiseAssets();

	InitDefaultFloor();

	particleBuffer = water->getparticleBuffer();
	renderer->setMarchingCubesBuffer(water->getTriangleBuffer());
	
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

	auto start_time
		= std::chrono::high_resolution_clock::now();

	water->Update(dt);

	auto physics_end_time
		= std::chrono::high_resolution_clock::now();

	auto taken_time_physics = std::chrono::duration_cast<
		std::chrono::milliseconds>(
			physics_end_time - start_time)
		.count();


	//positionList[1000] = positionList[1000] + Vector3(0, 0, 1) * dt;

	//ParticleObject->GetRenderObject()->GetMesh()->UpdateParticlesPositionInstance(positionList, numParticles);

	//OGLShader* shader = (OGLShader*)(ParticleObject->GetRenderObject()->GetShader());
	//shader->setParticleBuffer(particleBuffer);

	renderer->Update(dt);

	renderer->Render();

	Debug::UpdateRenderables(dt);
	auto render_end_time
		= std::chrono::high_resolution_clock::now();

	auto taken_time_render = std::chrono::duration_cast<
		std::chrono::milliseconds>(
			render_end_time - physics_end_time)
		.count();

	

	std::cout << "\r"
		<< "phy execution time: " << taken_time_physics
		<< "ms "
		<< "ren execution time: " << taken_time_render
		<< "ms ";
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

	//sphereMesh->SetInstanceModelMatrices(positionList, numParticles);

	return sphere;
}

void TutorialGame::InitDefaultFloor() {
	ParticleObject = AddSphereToWorld(Vector3(0, 0, 0), 0.5f);
}


