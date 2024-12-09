#pragma once
#include "Transform.h"
#include "CollisionVolume.h"
#include "../CSC8503/LayerMask.h"
#include <bitset>

using std::vector;

namespace NCL::CSC8503 {
	class NetworkObject;
	class RenderObject;
	class PhysicsObject;

	class GameObject	{
	public:
		GameObject(const std::string& name = "");
		~GameObject();

		void SetBoundingVolume(CollisionVolume* vol) {
			boundingVolume = vol;
		}

		const CollisionVolume* GetBoundingVolume() const {
			return boundingVolume;
		}

		bool IsActive() const {
			return isActive;
		}

		Transform& GetTransform() {
			return transform;
		}

		RenderObject* GetRenderObject() const {
			return renderObject;
		}

		PhysicsObject* GetPhysicsObject() const {
			return physicsObject;
		}

		NetworkObject* GetNetworkObject() const {
			return networkObject;
		}

		void SetRenderObject(RenderObject* newObject) {
			renderObject = newObject;
		}

		void SetPhysicsObject(PhysicsObject* newObject) {
			physicsObject = newObject;
		}

		const std::string& GetName() const {
			return name;
		}

		virtual void OnCollisionBegin(GameObject* otherObject) {
			//std::cout << "OnCollisionBegin event occured!\n";
		}

		virtual void OnCollisionEnd(GameObject* otherObject) {
			//std::cout << "OnCollisionEnd event occured!\n";
		}

		bool GetBroadphaseAABB(Vector3&outsize) const;

		void UpdateBroadphaseAABB();

		void SetWorldID(int newID) {
			worldID = newID;
		}

		int	GetWorldID() const {
			return worldID;
		}

		void SetLayer(Layer layer) { this->layer = layer; }
		Layer GetLayer() const { return layer; }

		void AddIgnoreLayer(Layer layer) {
			ignoreLayers.set(static_cast<size_t>(layer));
		}
		void RemoveIgnoreLayer(Layer layer) {
			ignoreLayers.reset(static_cast<size_t>(layer));
		}
		bool ShouldCollideWith(GameObject* other) {
			return !ignoreLayers.test(static_cast<size_t>(other->GetLayer()));
		}
		virtual bool ShouldResolvePhysicsCollisionWith(GameObject* other) {
			// 默认情况下，如果应该碰撞，就应该解析物理碰撞
			return ShouldCollideWith(other);
		}

	protected:
		Transform			transform;

		CollisionVolume*	boundingVolume;
		PhysicsObject*		physicsObject;
		RenderObject*		renderObject;
		NetworkObject*		networkObject;

		bool		isActive;
		int			worldID;
		std::string	name;

		Vector3 broadphaseAABB;

		Layer layer;
		std::bitset<static_cast<size_t>(Layer::MaxLayers)> ignoreLayers; //无视碰撞检测的Layer列表
	};
}

