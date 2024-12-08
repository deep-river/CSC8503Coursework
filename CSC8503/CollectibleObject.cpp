#include "CollectibleObject.h"
#include "PlayerObject.h"

using namespace NCL;
using namespace CSC8503;

CollectibleObject::CollectibleObject() {
}

CollectibleObject::~CollectibleObject() {
}

void CollectibleObject::Update(float dt) {
	GetTransform().SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0, 1, 0), rotationSpeed * dt));
}

void CollectibleObject::OnCollisionBegin(GameObject* otherObject) {
	std::cout << "Collided with " << otherObject->GetName() << std::endl;
	PlayerObject* player = dynamic_cast<PlayerObject*>(otherObject);
	if (player) {
		player->AddScore(1);
		isActive = false;
	}
}