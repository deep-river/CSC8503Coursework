#pragma once
#include "GameObject.h"

namespace NCL {
	namespace CSC8503 {
		class CollectibleObject : public GameObject {
		public:
			CollectibleObject();
			~CollectibleObject();

			void Update(float dt);
			void OnCollisionBegin(GameObject* otherObject) override;
		protected:
			float rotationSpeed = 2.0f; // Radians per second
		};
	}
}

