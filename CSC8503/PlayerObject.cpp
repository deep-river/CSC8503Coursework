#include "PlayerObject.h"

using namespace NCL;
using namespace CSC8503;

PlayerObject::PlayerObject(const std::string& objectName) : GameObject(objectName) {
	SetLayer(Layer::Player);
}

PlayerObject::~PlayerObject() {
}