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

			bool ShouldResolvePhysicsCollisionWith(GameObject* other) override {
				if (other->GetLayer() == Layer::Player) {
					return false;  // 不解析与Player的物理碰撞，只解析逻辑碰撞，即只执行OnCollisionBegin方法
				}
				return GameObject::ShouldResolvePhysicsCollisionWith(other);
			}

		protected:
			float rotationSpeed = 40.0f;
		};
	}
}

