#pragma once
#include "GameObject.h"
#include <GameWorld.h>

#define PI 3.14159265358979323846
#define RAD_TO_DEG (180.0 / PI)

namespace NCL {
    namespace CSC8503 {
        class StateMachine;
        class StateGameObject : public GameObject  {
        public:
            StateGameObject(GameWorld* world);
            ~StateGameObject();

            virtual void Update(float dt);

            void AddWaypoint(Vector3& waypoint);
			bool IsNearWaypoint(Vector3& point, float threshold = 10.0f);
			void MoveToWaypoint(float dt);

            void TurnToFace(Vector3& targetDirection);
            bool DetectPlayer(float detectionRange, float fanAngle);

        protected:
            void MoveLeft(float dt);
            void MoveRight(float dt);

            StateMachine* stateMachine;
            GameWorld* gameWorld;

            float counter;
            float moveSpeed;

            std::vector<Vector3> waypoints;
            size_t currentWaypointIndex;

            bool playerDetected;
        };
    }
}
