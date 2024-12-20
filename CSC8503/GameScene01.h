﻿#include "../NCLCoreClasses/KeyboardMouseController.h"

#pragma once
#include "GameTechRenderer.h"
#ifdef USEVULKAN
#include "GameTechVulkanRenderer.h"
#endif
#include "PhysicsSystem.h"
#include "PlayerObject.h"
#include "StateGameObject.h"
#include "CollectibleObject.h"

namespace NCL {
	namespace CSC8503 {
		class GameScene01 {
		public:
			GameScene01();
			~GameScene01();

			virtual void UpdateGame(float dt);
		protected:
			void InitialiseAssets();

			void InitCamera();

			void InitWorld();

			void InitTerrain(int width, int height, float cellSize);

			/*
			These are some of the world/object creation functions I created when testing the functionality
			in the module. Feel free to mess around with them to see different objects being created in different
			test scenarios (constraints, collision types, and so on).
			*/
			void InitGameObjects();

			void InitPlayer();

			void InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius);
			void InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing);
			void InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims);

			void InitDefaultFloor();

			GameObject* AddFloorToWorld(const Vector3& position);
			GameObject* AddSphereToWorld(const Vector3& position, float radius, float inverseMass = 10.0f, bool isHollow = false); //Added isHollow parameter, true = hollow object, false = solid object
			GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);
			GameObject* AddOBBCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);

			PlayerObject* AddPlayerToWorld(const Vector3& position);
			StateGameObject* AddStateObjectToWorld(const Vector3& position);
			GameObject* AddEnemyToWorld(const Vector3& position);
			StateGameObject* AddGooseToWorld(const Vector3& position);
			CollectibleObject* AddBonusToWorld(const Vector3& position);

			void BridgeConstraintTest();

			void UpdatePlayer(float dt);
			void UpdateCamera(float dt);
			void UpdateGameUI();
			void RenderMenu();
			void RenderGameOverScreen();
			void UpdateGameTimer(float dt);
			void UpdateKeys();

			void UpdateCollectibles(float dt);

			void StartLevel();

#ifdef USEVULKAN
			GameTechVulkanRenderer* renderer;
#else
			GameTechRenderer* renderer;
#endif
			PhysicsSystem* physics;
			GameWorld* world;

			KeyboardMouseController controller;

			bool useGravity;
			bool inSelectionMode;

			GameObject* selectionObject = nullptr;

			Mesh* capsuleMesh = nullptr;
			Mesh* cubeMesh = nullptr;
			Mesh* sphereMesh = nullptr;

			Texture* basicTex = nullptr;
			Shader* basicShader = nullptr;

			//Coursework Meshes
			Mesh* catMesh = nullptr;
			Mesh* kittenMesh = nullptr;
			Mesh* enemyMesh = nullptr;
			Mesh* bonusMesh = nullptr;
			Mesh* coinMesh = nullptr;
			Mesh* gooseMesh = nullptr;

			Vector3 lockedOffset = Vector3(0, 5, -10); //相机对跟踪对象的位置偏移量
			float cameraDistance = -10.0f; //相机与跟踪对象的距离
			float cameraHeight = 5.0f; //相机高度

			//PlayerObject* player;
			PlayerObject* localPlayer;

			float playerMoveSpeed = 2.0f; //角色移动速度
			Vector3 playerSpawnPos = Vector3(-160, 5, 0); //角色生成位置

			float gameTimer; //游戏剩余时间
			float gameDuration = 75.0f; //每局游戏持续时间
			//int playerScore = 0; //玩家分数
			bool isGameOver = false;
			bool showMenu = false; 

			StateGameObject* simplePatrolObject;
			StateGameObject* gooseObject;
			
			std::vector<CollectibleObject*> collectibles;

			Vector4 playerColour = Vector4(1.0f, 0.65f, 0.18f, 1.0f); //角色颜色_橙色
			Vector4 collectibleColour = Vector4(0.97f, 0.97f, 0.18f, 1.0f); //角色颜色_橙色
		};
	}
}


