#include "StateGameObject.h"
#include "StateTransition.h"
#include "StateMachine.h"
#include "State.h"
#include "PhysicsObject.h"

using namespace NCL;
using namespace CSC8503;

StateGameObject::StateGameObject() {
	counter = 0.0f;
	moveSpeed = 1500.0f;
	waypoints.clear();
	currentWaypointIndex = 0;

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
	State* stateMoveAlongWaypoints = new State([&](float dt)->void {
		this->MoveToWaypoint(dt);
		});

	stateMachine->AddState(stateMoveAlongWaypoints);

	// Add a transition to cycle through waypoints
	/*stateMachine->AddTransition(new StateTransition(stateMoveAlongWaypoints, stateMoveAlongWaypoints,
		[&]()->bool {
			return this->IsNearWaypoint(waypoints[currentWaypointIndex], 1.0f);
		}
	));*/
}

StateGameObject::~StateGameObject() {
	delete stateMachine;
}

void StateGameObject::Update(float dt) {
	GetPhysicsObject()->ClearForces();
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
	if (waypoints.empty()) {
		std::cout << "Waypoints vector is empty! Exit move mode." << std::endl;
		return;
	}

	Vector3 currentPosition = GetTransform().GetPosition();
	Vector3 targetWaypoint = waypoints[currentWaypointIndex];
	Vector3 toWaypoint = targetWaypoint - currentPosition;

	if (IsNearWaypoint(targetWaypoint, 5.0f)) {
		currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.size();
		std::cout << "Simple patrol unit reached waypoint " << currentWaypointIndex << ". Moving to next waypoint." << std::endl;
		// Recalculate toWaypoint for the new target
		targetWaypoint = waypoints[currentWaypointIndex];
		toWaypoint = targetWaypoint - currentPosition;

	}

	Vector3 direction = Vector::Normalise(toWaypoint);
	GetPhysicsObject()->AddForce(direction * moveSpeed * dt);
}

void StateGameObject::TurnAround() {
	Quaternion currentOrientation = GetTransform().GetOrientation();
	Quaternion newOrientation = Quaternion::AxisAngleToQuaterion(Vector3(0, 1, 0), 180);
	GetTransform().SetOrientation(currentOrientation * newOrientation);
}