#include "CollectibleObject.h"
#include "PlayerObject.h"

using namespace NCL;
using namespace CSC8503;

CollectibleObject::CollectibleObject() {
	SetLayer(Layer::Collectible);
}

CollectibleObject::~CollectibleObject() {
}

void CollectibleObject::Update(float dt) {
	GetTransform().SetOrientation(GetTransform().GetOrientation() * Quaternion::AxisAngleToQuaterion(Vector3(0, 1, 0), rotationSpeed * dt));
}

void CollectibleObject::OnCollisionBegin(GameObject* otherObject) {
	//std::cout << "Collided with " << otherObject->GetName() << std::endl;
	if (otherObject->GetLayer() == Layer::Player) {
		// cast otherObject to PlayerObject
		PlayerObject* player = dynamic_cast<PlayerObject*>(otherObject);
		player->AddScore(1);
		isActive = false;
	}
}
