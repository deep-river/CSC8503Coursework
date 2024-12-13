#pragma once
#include "GameObject.h"
#include <GameWorld.h>
#include <State.h>

namespace NCL {
    namespace CSC8503 {
        class StateMachine;
        class StateGameObject : public GameObject  {
        public:
            StateGameObject(GameWorld* world);
            ~StateGameObject();

            virtual void Update(float dt);

            void AddWaypoint(Vector3& waypoint);
			void MoveToWaypoint(float dt);

            void TurnToFace(Vector3& targetDirection);
			void StopMoving();
            bool DetectPlayer(float detectionRange, float fanAngle);
            void ChasePlayer(float dt);

        protected:
            const float RAD_TO_DEG = 57.295779513f;

            void MoveLeft(float dt);
            void MoveRight(float dt);

            StateMachine* stateMachine;
            GameWorld* gameWorld;

            float counter;
            float moveSpeed;

            std::vector<Vector3> waypoints;
            size_t currentWaypointIndex;

            bool playerDetected;
            Vector3 playerPosition;
            float chaseTimer;
            float maxChaseTime;
            float detectionRange;

            State* patrolState;
            State* alertState;

            bool IsNearWaypoint(Vector3& point, float threshold = 10.0f);
        };
    }
}
