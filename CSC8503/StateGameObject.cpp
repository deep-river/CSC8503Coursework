#include "StateGameObject.h"
#include "StateTransition.h"
#include "StateMachine.h"
#include "State.h"
#include "PhysicsObject.h"
#include "Ray.h"
#include "Debug.h"

using namespace NCL;
using namespace CSC8503;

StateGameObject::StateGameObject(GameWorld* world) {
	counter = 0.0f;
	moveSpeed = 1500.0f;
	waypoints.clear();
	currentWaypointIndex = 0;
	playerDetected = false;
	gameWorld = world;

	stateMachine = new StateMachine();

	/*State* stateA = new State([&](float dt)->void {
		this->MoveLeft(dt);
	});
	State* stateB = new State([&](float dt)->void {
		this->MoveRight(dt);
	});

	stateMachine->AddState(stateA);
	stateMachine->AddState(stateB);

	stateMachine->AddTransition(new StateTransition(stateA, stateB, [&]()->bool {
		return this->counter > 8.0f;
	}));
	stateMachine->AddTransition(new StateTransition(stateB, stateA, [&]()->bool {
		return this->counter < -8.0f;
	}));

	State* stateMoveAlongWaypoints = new State([&](float dt)->void {
		this->MoveToWaypoint(dt);
	});*/
	State* patrolState = new State([&](float dt)->void {
		this->MoveToWaypoint(dt);
		//Debug::Print("Patrol State", Vector2(10, 30));
		});
	State* alertState = new State([&](float dt) -> void {
		// Stop moving and perform alert behavior
		GetPhysicsObject()->ClearForces();
		GetPhysicsObject()->SetLinearVelocity(Vector3(0, 0, 0));
		//Debug::Print("Alert State", Vector2(10, 30));
		});

	stateMachine->AddState(patrolState);
	stateMachine->AddState(alertState);

	stateMachine->AddTransition(new StateTransition(patrolState, alertState,
		[&]() -> bool {
			return playerDetected;
		}));

	stateMachine->AddTransition(new StateTransition(alertState, patrolState,
		[&]() -> bool {
			return !playerDetected;
		}));
}

StateGameObject::~StateGameObject() {
	delete stateMachine;
}

void StateGameObject::Update(float dt) {
	GetPhysicsObject()->ClearForces();

	playerDetected = DetectPlayer(50.0f, 45.0f);
	stateMachine->Update(dt);
}

void StateGameObject::MoveLeft(float dt) {
	GetPhysicsObject()->AddForce({ 0,0,-7 });
	counter += dt;
}

void StateGameObject::MoveRight(float dt) {
	GetPhysicsObject()->AddForce({ 0,0,7 });
	counter -= dt;
}

void StateGameObject::AddWaypoint(Vector3& waypoint) {
	waypoints.push_back(waypoint);
}

bool StateGameObject::IsNearWaypoint(Vector3& point, float threshold) {
	Vector3 toPoint = point - GetTransform().GetPosition();
	return Vector::Length(toPoint) < threshold;
}

void StateGameObject::MoveToWaypoint(float dt) {
	if (waypoints.empty() || playerDetected) {
		std::cout << "Waypoints vector is empty! Exit move mode." << std::endl;
		return;
	}

	Vector3 currentPosition = GetTransform().GetPosition();
	Vector3 targetWaypoint = waypoints[currentWaypointIndex];
	Vector3 toWaypoint = targetWaypoint - currentPosition;

	if (IsNearWaypoint(targetWaypoint, 5.0f)) {
		currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.size();
		//std::cout << "Simple patrol unit reached waypoint " << currentWaypointIndex << ". Moving to next waypoint." << std::endl;
		targetWaypoint = waypoints[currentWaypointIndex];
		toWaypoint = targetWaypoint - currentPosition;

	}

	Vector3 direction = Vector::Normalise(toWaypoint);
	// Turn to face the direction of movement
	//TurnToFace(direction);
	GetPhysicsObject()->AddForce(direction * moveSpeed * dt);
}

void StateGameObject::TurnToFace(Vector3& targetDirection) {
	Vector3 currentForward = GetTransform().GetOrientation() * Vector3(0, 0, -1);
	//Vector3 desiredForward = Vector::Normalise(targetDirection);

	// 计算旋转轴和角度
	Vector3 rotationAxis = Vector::Normalise(Vector::Cross(currentForward, targetDirection));
	float rotationAngle = acos(Vector::Dot(currentForward, targetDirection));

	// 创建旋转四元数
	Quaternion rotationQuaternion = Quaternion::AxisAngleToQuaterion(rotationAxis, rotationAngle * (180.0 / 3.14159265358979323846));

	// 应用旋转
	GetTransform().SetOrientation(GetTransform().GetOrientation() * rotationQuaternion);
}


bool StateGameObject::DetectPlayer(float detectionRange, float fanAngle) {
	Vector3 forward = GetTransform().GetOrientation() * Vector3(0, 0, -1);
	Vector3 position = GetTransform().GetPosition() + Vector3(0, -2, 0);

	const int numRays = 5;
	const float angleStep = fanAngle / (numRays - 1);

	for (int i = 0; i < numRays; ++i) {
		float currentAngle = -fanAngle / 2 + i * angleStep;
		Quaternion rotation = Quaternion::AxisAngleToQuaterion(Vector3(0, 1, 0), currentAngle);
		Vector3 rayDirection = rotation * forward;

		Ray ray(position, rayDirection);
		RayCollision closestCollision;

		Debug::DrawLine(position, position + rayDirection * detectionRange, Debug::RED, 0.02f);

		if (gameWorld->Raycast(ray, closestCollision, true, this)) {
			GameObject* hitObject = (GameObject*)closestCollision.node;
			//if (hitObject->GetName() != "") std::cout << "Ray hit object: " << hitObject->GetName() << std::endl;
			if (hitObject->GetName() == "Player" && closestCollision.rayDistance <= detectionRange) {
				//std::cout << "Player detected!" << std::endl;
				return true;
			}
		}
	}
	return false;
}