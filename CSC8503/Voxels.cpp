#include "Voxels.h"
#include "TutorialGame.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "Player.h"

#include <iostream>

using namespace NCL::CSC8503;

Voxels::Voxels(const std::string& objectName)
{
	tag = "Voxel";
	name = objectName;
	worldID = -1;
	isActive = true;
	boundingVolume = nullptr;
	physicsObject = nullptr;
	renderObject = nullptr;
	networkObject = nullptr;
	isDestroyed = false;
}

//void Voxels::OnCollisionBegin(GameObject* otherObject)
//{
//	std::cout << "why am i here";
//	if (otherObject->gettag() == "Player") {
//		Player* player = dynamic_cast<Player*>(otherObject);
//		if (player->getDestroyVoxels()) {
//			deactivate();
//			isDestroyed = true;
//			std::cout << "I should be here";
//
//			//UpdateVoxels();
//			//player->AddScore(0);
//		}
//	}
//}

