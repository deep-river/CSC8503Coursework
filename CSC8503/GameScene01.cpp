﻿#include "GameScene01.h"
#include "GameWorld.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "TextureLoader.h"

#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "StateGameObject.h"
#include "CollectibleObject.h"



using namespace NCL;
using namespace CSC8503;

GameScene01::GameScene01() : controller(*Window::GetWindow()->GetKeyboard(), *Window::GetWindow()->GetMouse()) {
	world = new GameWorld();
#ifdef USEVULKAN
	renderer = new GameTechVulkanRenderer(*world);
	renderer->Init();
	renderer->InitStructures();
#else 
	renderer = new GameTechRenderer(*world);
#endif

	physics = new PhysicsSystem(*world);
	useGravity = true;

	gameTimer = gameDuration;
	isGameOver = false;
	showMenu = false;

	world->GetMainCamera().SetController(controller);

	controller.MapAxis(0, "Sidestep");
	controller.MapAxis(1, "UpDown");
	controller.MapAxis(2, "Forward");
	controller.MapAxis(3, "XLook");
	controller.MapAxis(4, "YLook");

	simplePatrolObject = nullptr;
	InitialiseAssets();
	std::cout << ">>>>>>>>>>[Scene: GameScene01 begin!]<<<<<<<<<<" << std::endl;

	//StartLevel();
}

void GameScene01::InitialiseAssets() {
	cubeMesh = renderer->LoadMesh("cube.msh");
	sphereMesh = renderer->LoadMesh("sphere.msh");
	catMesh = renderer->LoadMesh("ORIGAMI_Chat.msh");
	kittenMesh = renderer->LoadMesh("Kitten.msh");

	enemyMesh = renderer->LoadMesh("Keeper.msh");
	bonusMesh = renderer->LoadMesh("19463_Kitten_Head_v1.msh");
	capsuleMesh = renderer->LoadMesh("capsule.msh");
	gooseMesh = renderer->LoadMesh("goose.msh");

	coinMesh = renderer->LoadMesh("coin.msh");

	basicTex = renderer->LoadTexture("checkerboard.png");
	basicShader = renderer->LoadShader("scene.vert", "scene.frag");

}

GameScene01::~GameScene01() {
	delete cubeMesh;
	delete sphereMesh;
	delete catMesh;
	delete kittenMesh;
	delete enemyMesh;
	delete bonusMesh;

	delete basicTex;
	delete basicShader;

	delete localPlayer;
	delete simplePatrolObject;
	
	for (auto collectible : collectibles) {
		delete collectible;
	}
	collectibles.clear();

	delete physics;
	delete renderer;
	delete world;
}

//初始化单机场景
void GameScene01::StartLevel() {
	InitWorld();
	InitCamera();
	InitGameObjects();
	InitPlayer();
}

void GameScene01::InitCamera() {
	world->GetMainCamera().SetNearPlane(0.1f);
	world->GetMainCamera().SetFarPlane(500.0f);
	world->GetMainCamera().SetPitch(-15.0f);
	world->GetMainCamera().SetYaw(315.0f);
	world->GetMainCamera().SetPosition(Vector3(-60, 40, 60));
}

void GameScene01::InitWorld() {
	world->ClearAndErase();
	physics->Clear();
	//InitMixedGridWorld(15, 15, 3.5f, 3.5f);
	//BridgeConstraintTest(); //重力吊桥物理约束测试
	//InitDefaultFloor();
	InitTerrain(20, 20, 10.0f);
	//InitGameObjects();
}

void GameScene01::InitDefaultFloor() {
	AddFloorToWorld(Vector3(0, -20, 0));
}

void GameScene01::InitTerrain(int width, int height, float cellSize) {
	AddFloorToWorld(Vector3(0, -1, 0));

	// Define the size of a single terrain block
	float blockSize = 10.0f;

	// Create terrain blocks
	// 1. Create a U-shaped path
	for (int i = 0; i < 10; ++i) {
		AddCubeToWorld(Vector3(0 + i * blockSize, 5, -40), Vector3(blockSize / 2, 10, blockSize / 2), 0)->GetRenderObject()->SetColour(Vector4(0.5f, 0.5f, 0.5f, 1.0f));
		AddCubeToWorld(Vector3(0 + i * blockSize, 5, 40), Vector3(blockSize / 2, 10, blockSize / 2), 0)->GetRenderObject()->SetColour(Vector4(0.5f, 0.5f, 0.5f, 1.0f));
	}
	for (int i = 0; i < 8; ++i) {
		AddCubeToWorld(Vector3(90, 5, -30 + i * blockSize), Vector3(blockSize / 2, 10, blockSize / 2), 0)->GetRenderObject()->SetColour(Vector4(0.5f, 0.5f, 0.5f, 1.0f));
	}

	// 2. Create some obstacles and decorative elements
	AddCubeToWorld(Vector3(40, 7.5, 0), Vector3(blockSize, 15, blockSize), 0)->GetRenderObject()->SetColour(Vector4(0.7f, 0.3f, 0.3f, 1.0f));
	AddCubeToWorld(Vector3(10, 10, 0), Vector3(blockSize, 20, blockSize), 0)->GetRenderObject()->SetColour(Vector4(0.3f, 0.7f, 0.3f, 1.0f));
	AddCubeToWorld(Vector3(70, 12.5, 0), Vector3(blockSize, 25, blockSize), 0)->GetRenderObject()->SetColour(Vector4(0.3f, 0.3f, 0.7f, 1.0f));

	// 3. Create open areas for items
	// Left open area
	AddCubeToWorld(Vector3(10, 2.5, -30), Vector3(blockSize * 1.0, 5, blockSize * 1.0), 0)->GetRenderObject()->SetColour(Vector4(0.7f, 0.7f, 0.3f, 1.0f));
	// Right open area
	AddCubeToWorld(Vector3(70, 2.5, 30), Vector3(blockSize * 1.0, 5, blockSize * 1.0), 0)->GetRenderObject()->SetColour(Vector4(0.3f, 0.7f, 0.7f, 1.0f));

	//出生点走道与背后围墙
	AddCubeToWorld(Vector3(-140, 5, -50), Vector3(blockSize * 6, 5, blockSize * 4), 0);
	AddCubeToWorld(Vector3(-140, 5, 50), Vector3(blockSize * 6, 5, blockSize * 4), 0);
	AddCubeToWorld(Vector3(-180, 5, 0), Vector3(blockSize * 0.5, 5, blockSize * 4), 0);

	AddCubeToWorld(Vector3(-50, 5, -50), Vector3(blockSize * 2, 5, blockSize * 4), 0);
	AddCubeToWorld(Vector3(-50, 5, 50), Vector3(blockSize * 2, 5, blockSize * 4), 0);
}

void GameScene01::InitGameObjects() {
	//门
	GameObject* doorLeft = AddCubeToWorld(Vector3(-150, 3, -4), Vector3(1, 2, 4), 60);
	doorLeft->GetRenderObject()->SetColour(Debug::GREEN);
	GameObject* posLeft = AddCubeToWorld(Vector3(-150, 3, -9), Vector3(1, 2, 1), 0);

	OrientationConstraint* orientationConstraintL = new OrientationConstraint(posLeft, doorLeft, PI / 8.0f); //旋转约束为90度
	PositionConstraint* positionConstraintL = new PositionConstraint(posLeft, doorLeft, 5);
	world->AddConstraint(positionConstraintL);
	world->AddConstraint(orientationConstraintL);

	GameObject* doorRight = AddCubeToWorld(Vector3(-150, 3, 4), Vector3(1, 2, 4), 60);
	doorRight->GetRenderObject()->SetColour(Debug::GREEN);
	GameObject* posRight = AddCubeToWorld(Vector3(-150, 3, 9), Vector3(1, 2, 1), 0);

	OrientationConstraint* orientationConstraintR = new OrientationConstraint(posRight, doorRight, PI / 8.0f); //旋转约束为90度
	PositionConstraint* positionConstraintR = new PositionConstraint(posRight, doorRight, 5);
	world->AddConstraint(positionConstraintR);
	world->AddConstraint(orientationConstraintR);

	//推箱子 OBB vs Sphere(player volume) collision
	AddOBBCubeToWorld(Vector3(-120, 15, -6), Vector3(2, 2, 2), 1)->GetRenderObject()->SetColour(Debug::GREEN);
	AddOBBCubeToWorld(Vector3(-120, 10, -2), Vector3(2, 2, 2), 1)->GetRenderObject()->SetColour(Debug::GREEN);
	AddOBBCubeToWorld(Vector3(-120, 5, 2), Vector3(2, 2, 2), 1)->GetRenderObject()->SetColour(Debug::GREEN);
	AddOBBCubeToWorld(Vector3(-120, 5, 6), Vector3(2, 2, 2), 1)->GetRenderObject()->SetColour(Debug::GREEN);

	//简单巡逻敌人，巡逻路径为(-75, 2, -80) - (-75, 2, 80)
	simplePatrolObject = AddStateObjectToWorld(Vector3(-75, 5, 0));
	Vector3 waypoint1 = Vector3(-75, 0, 80);
	Vector3 waypoint2 = Vector3(-75, 0, -80);
	simplePatrolObject->AddWaypoint(waypoint1);
	simplePatrolObject->AddWaypoint(waypoint2);
	//goose，巡逻路径为(-10, 1.5, -15) - (25, 1.5, -15) - (25, 1.5, 20) - (-10, 1.5, 20)
	gooseObject = AddGooseToWorld(Vector3(-10, 1.5, 20));
	Vector3 gWaypoint1 = Vector3(-10, 1.5, -15);
	Vector3 gWaypoint2 = Vector3(25, 1.5, -15);
	Vector3 gWaypoint3 = Vector3(25, 1.5, 20);
	Vector3 gWaypoint4 = Vector3(-10, 1.5, 20);
	gooseObject->AddWaypoint(gWaypoint1);
	gooseObject->AddWaypoint(gWaypoint2);
	gooseObject->AddWaypoint(gWaypoint3);
	gooseObject->AddWaypoint(gWaypoint4);

	AddBonusToWorld(Vector3(-70, 5, 0));
	AddBonusToWorld(Vector3(-80, 5, 0));
	AddBonusToWorld(Vector3(-90, 5, 0));
	AddBonusToWorld(Vector3(-100, 5, 0));
	AddBonusToWorld(Vector3(-110, 5, 0));

	AddBonusToWorld(Vector3(-10, 5, -25));
	AddBonusToWorld(Vector3(-10, 5, -15));
	AddBonusToWorld(Vector3(-10, 5, -5));
	AddBonusToWorld(Vector3(-10, 5, 5));
	AddBonusToWorld(Vector3(-10, 5, 15));
	AddBonusToWorld(Vector3(-10, 5, 25));

	AddBonusToWorld(Vector3(0, 5, 20));
	AddBonusToWorld(Vector3(10, 5, 20));
	AddBonusToWorld(Vector3(20, 5, 20));
	AddBonusToWorld(Vector3(30, 5, 20));

	AddBonusToWorld(Vector3(0, 5, -15));
	AddBonusToWorld(Vector3(10, 5, -15));
	AddBonusToWorld(Vector3(20, 5, -15));
	AddBonusToWorld(Vector3(30, 5, -15));
}

void GameScene01::InitPlayer() {
	localPlayer = AddPlayerToWorld(playerSpawnPos);
}

void GameScene01::InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddSphereToWorld(position, radius, 1.0f);
		}
	}
	AddFloorToWorld(Vector3(0, -2, 0));
}

void GameScene01::InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing) {
	float sphereRadius = 1.0f;
	Vector3 cubeDims = Vector3(1, 1, 1);

	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);

			if (rand() % 2) {
				AddCubeToWorld(position, cubeDims);
			}
			else {
				AddSphereToWorld(position, sphereRadius);
			}
		}
	}
}

void GameScene01::InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims) {
	for (int x = 1; x < numCols + 1; ++x) {
		for (int z = 1; z < numRows + 1; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddCubeToWorld(position, cubeDims, 1.0f);
		}
	}
}

void GameScene01::BridgeConstraintTest() {
	std::cout << "Bridge constraint test begin" << std::endl;
	Vector3 cubeSize = Vector3(8, 8, 8);

	float invCubeMass = 5;
	int numLinks = 10;
	float maxDistance = 30;
	float cubeDistance = 20;

	Vector3 startPos = Vector3(-120, 120, 0);

	GameObject* start = AddCubeToWorld(startPos + Vector3(0, 0, 0), cubeSize, 0);
	GameObject* end = AddCubeToWorld(startPos + Vector3((numLinks + 2) * cubeDistance, 0, 0), cubeSize, 0);
	GameObject* previous = start;

	for (int i = 0; i < numLinks; ++i) {
		GameObject* block = AddCubeToWorld(startPos + Vector3((i + 1) * cubeDistance, 0, 0), cubeSize, invCubeMass);
		PositionConstraint* positionConstraint = new PositionConstraint(previous, block, maxDistance);
		OrientationConstraint* orientationConstraint = new OrientationConstraint(previous, block, PI / 4.0f); //旋转约束为45度

		world->AddConstraint(positionConstraint);
		world->AddConstraint(orientationConstraint);
		previous = block;
	}
	PositionConstraint* positionConstraint = new PositionConstraint(previous, end, maxDistance);
	OrientationConstraint* orientationConstraint = new OrientationConstraint(previous, end, PI / 4.0f); //旋转约束为45度

	world->AddConstraint(positionConstraint);
	world->AddConstraint(orientationConstraint);
}

GameObject* GameScene01::AddFloorToWorld(const Vector3& position) {
	GameObject* floor = new GameObject();

	Vector3 floorSize = Vector3(200, 2, 200);
	AABBVolume* volume = new AABBVolume(floorSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform().SetScale(floorSize * 2.0f).SetPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));
	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();
	world->AddGameObject(floor);

	return floor;
}

GameObject* GameScene01::AddSphereToWorld(const Vector3& position, float radius, float inverseMass, bool isHollow) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);
	sphere->GetTransform().SetScale(sphereSize).SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	//生成空心或实心球体
	if (isHollow) {
		sphere->GetPhysicsObject()->InitHollowSphereInertia();
	}
	else {
		sphere->GetPhysicsObject()->InitSphereInertia();
	}
	world->AddGameObject(sphere);

	return sphere;
}

GameObject* GameScene01::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);
	cube->GetTransform().SetPosition(position).SetScale(dimensions * 2.0f);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));
	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();
	world->AddGameObject(cube);

	return cube;
}

GameObject* GameScene01::AddOBBCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	OBBVolume* volume = new OBBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);
	cube->GetTransform().SetPosition(position).SetScale(dimensions * 2.0f);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));
	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();
	world->AddGameObject(cube);

	return cube;
}

PlayerObject* GameScene01::AddPlayerToWorld(const Vector3& position) {
	float meshSize = 1.0f;
	float inverseMass = 10.0f;

	PlayerObject* character = new PlayerObject("Player");
	SphereVolume* volume = new SphereVolume(0.5f);

	character->SetBoundingVolume((CollisionVolume*)volume);
	character->GetTransform().SetScale(Vector3(meshSize, meshSize, meshSize)).SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), catMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));
	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();
	character->GetRenderObject()->SetColour(playerColour);

	character->SetCaught(false);

	world->AddGameObject(character);
	return character;
}

StateGameObject* GameScene01::AddStateObjectToWorld(const Vector3& position) {
	StateGameObject* apple = new StateGameObject(world);

	SphereVolume* volume = new SphereVolume(2.5f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(4, 4, 4))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), enemyMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();
	apple->GetRenderObject()->SetColour(Debug::RED);

	world->AddGameObject(apple);

	return apple;
}

GameObject* GameScene01::AddEnemyToWorld(const Vector3& position) {
	float meshSize = 3.0f;
	float inverseMass = 0.5f;

	GameObject* character = new GameObject();

	AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);
	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform().SetScale(Vector3(meshSize, meshSize, meshSize)).SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), enemyMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));
	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();
	world->AddGameObject(character);

	return character;
}

StateGameObject* GameScene01::AddGooseToWorld(const Vector3& position) {
	StateGameObject* apple = new StateGameObject(world);

	SphereVolume* volume = new SphereVolume(1.0f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(1, 1, 1))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), gooseMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();
	apple->GetRenderObject()->SetColour(Debug::RED);

	world->AddGameObject(apple);

	return apple;
}

CollectibleObject* GameScene01::AddBonusToWorld(const Vector3& position) {
	CollectibleObject* item = new CollectibleObject();

	SphereVolume* volume = new SphereVolume(1.0f);
	item->SetBoundingVolume((CollisionVolume*)volume);
	item->GetTransform().SetScale(Vector3(0.2, 0.2, 0.2)).SetPosition(position);

	item->SetRenderObject(new RenderObject(&item->GetTransform(), coinMesh, nullptr, basicShader));
	item->SetPhysicsObject(new PhysicsObject(&item->GetTransform(), item->GetBoundingVolume()));
	item->GetPhysicsObject()->SetInverseMass(1.0f);
	item->GetPhysicsObject()->InitSphereInertia();

	//item->AddIgnoreLayer(Layer::Player);
	item->GetRenderObject()->SetColour(collectibleColour);
	world->AddGameObject(item);
	collectibles.push_back(item);
	return item;
}

// update functions

void GameScene01::UpdateGame(float dt) {
	if (!isGameOver) {
		if (showMenu) {
			RenderMenu();
		}
		else {
			UpdateGameTimer(dt);
			physics->UseGravity(useGravity);
			physics->Update(dt);
			world->UpdateWorld(dt);

			UpdatePlayer(dt);
			UpdateCamera(dt);
			UpdateGameUI();

			if (simplePatrolObject) {
				simplePatrolObject->Update(dt);
			}
			if (gooseObject) {
				gooseObject->Update(dt);
			}

			UpdateCollectibles(dt);
		}
	} else {
		RenderGameOverScreen();
	}

	UpdateKeys();
	renderer->Update(dt);
	renderer->Render();
	Debug::UpdateRenderables(dt);
}

void GameScene01::UpdateKeys() {
	if (Window::GetKeyboard()->KeyPressed(KeyCodes::ESCAPE)) {
		showMenu = !showMenu;
	}
}

void GameScene01::UpdateGameTimer(float dt) {
	gameTimer -= dt;
	if (gameTimer <= 0) {
		isGameOver = true;
	}
}

void GameScene01::UpdatePlayer(float dt) {
	if (!localPlayer) return;

	if (localPlayer->IsCaught()) {
		//localPlayer->GetTransform().SetPosition(playerSpawnPos);
		isGameOver = true;
	}
	/*float yaw = Window::GetMouse()->GetRelativePosition().x * 100.0f;
	localPlayer->GetTransform().SetOrientation(localPlayer->GetTransform().GetOrientation() * Quaternion::AxisAngleToQuaterion(Vector3(0, 1, 0), -yaw * dt));*/

	// Use angular impulse to rotate the player
	Vector3 rotationAxis = Vector3(0, 1, 0);
	float yaw = Window::GetMouse()->GetRelativePosition().x * 1.0f;
	float rotationStrength = 0.1f; // Adjust this value to control rotation speed
	Vector3 angularImpulse = -rotationAxis * yaw * rotationStrength * dt;
	localPlayer->GetPhysicsObject()->ApplyAngularImpulse(angularImpulse);

	Quaternion objOrientation = localPlayer->GetTransform().GetOrientation();
	Vector3 forward = objOrientation * Vector3(0, 0, -1);
	Vector3 right = objOrientation * Vector3(1, 0, 0);
	//player movement
	Vector3 movement(0, 0, 0);
	if (Window::GetKeyboard()->KeyDown(KeyCodes::W)) {
		movement -= forward;
	}
	if (Window::GetKeyboard()->KeyDown(KeyCodes::S)) {
		movement += forward;
	}
	if (Window::GetKeyboard()->KeyDown(KeyCodes::A)) {
		movement += right;
	}
	if (Window::GetKeyboard()->KeyDown(KeyCodes::D)) {
		movement -= right;
	}

	if (Vector::Length(movement) == 0) {
		//当输入的合成方向为0，即玩家松开按键时，将角色的水平速度设置为零，避免松开按键后仍惯性移动
		//注意Y轴速度保持不变，避免角色不受重力影响悬空
		float currrentYSpeed = localPlayer->GetPhysicsObject()->GetLinearVelocity().y;
		localPlayer->GetPhysicsObject()->SetLinearVelocity(Vector3(0, currrentYSpeed, 0));
	} else if (Vector::Length(movement) > 0) {
		// Normalize and scale the movement
		Vector::Normalise(movement);
		movement *= playerMoveSpeed * dt;
		localPlayer->GetPhysicsObject()->ApplyLinearImpulse(movement);
	}
	
}

void GameScene01::UpdateCamera(float dt) {
	if (!localPlayer) return;

	Quaternion objOrientation = localPlayer->GetTransform().GetOrientation();
	//鼠标滚轮调整相机与角色之间的跟踪距离
	float moveWheelMovement = Window::GetMouse()->GetWheelMovement();
	cameraDistance += moveWheelMovement * 1.0f;
	cameraDistance = std::min(-5.0f, std::max(cameraDistance, -15.0f)); // Clamp
	cameraHeight -= moveWheelMovement * 0.5f;
	cameraHeight = std::max(2.5f, std::min(cameraHeight, 7.5f)); // Clamp

	lockedOffset.y = cameraHeight;
	lockedOffset.z = cameraDistance;

	Vector3 objPos = localPlayer->GetTransform().GetPosition();
	Vector3 camPos = objPos + (objOrientation * lockedOffset);

	world->GetMainCamera().UpdateCamera(dt);
	world->GetMainCamera().SetPosition(camPos);
	//world->GetMainCamera().SetPitch(0); //锁定摄像机Y轴移动
	world->GetMainCamera().SetYaw(localPlayer->GetTransform().GetOrientation().ToEuler().y + 180.0f);
}

void GameScene01::UpdateGameUI() {
	if (!isGameOver) {
		Debug::Print("GameScene01", Vector2(50, 5), Debug::WHITE);
		Debug::Print("Press (ESC) to toggle menu", Vector2(50, 10), Debug::WHITE);
		Debug::Print("Time left: " + std::to_string(static_cast<int>(gameTimer)), Vector2(2, 5), Debug::YELLOW);
		Debug::Print("Score: " + std::to_string(localPlayer->GetScore()), Vector2(2, 10), Debug::YELLOW);

        Vector3 playerPos = localPlayer->GetTransform().GetPosition();
        std::string playerPosStr = "Player Position: (" + std::to_string(playerPos.x) + ", " + std::to_string(playerPos.y) + ", " + std::to_string(playerPos.z) + ")";
        Debug::Print(playerPosStr, Vector2(2, 100), Debug::WHITE);
	}
}

void GameScene01::RenderMenu() {
	Debug::Print("Menu", Vector2(40, 40), Debug::WHITE);
	Debug::Print("1. Play Again", Vector2(40, 45), Debug::WHITE);
	Debug::Print("2. Exit Game", Vector2(40, 50), Debug::WHITE);

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::NUM1)) {
		// Reset game state
		gameTimer = gameDuration;
		localPlayer->ResetScore();
		isGameOver = false;
		showMenu = false;
		//todo: 初始化networked game and fix bug
		//InitWorld();
		StartLevel();
	}
	else if (Window::GetKeyboard()->KeyPressed(KeyCodes::NUM2)) {
		exit(0);
	}
}

void GameScene01::RenderGameOverScreen() {
	Debug::Print("Game Over!", Vector2(40, 35), Debug::RED);
	Debug::Print("Final Score: " + std::to_string(localPlayer->GetScore()), Vector2(40, 40), Debug::YELLOW);
	Debug::Print("1. Play Again", Vector2(40, 50), Debug::WHITE);
	Debug::Print("2. Exit Game", Vector2(40, 55), Debug::WHITE);

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::NUM1)) {
		gameTimer = gameDuration;
		localPlayer->ResetScore();
		isGameOver = false;
		showMenu = false;
		//todo: 初始化networked game and fix bug
		//InitWorld();
		StartLevel();
	}
	else if (Window::GetKeyboard()->KeyPressed(KeyCodes::NUM2)) {
		exit(0);
	}
}

void GameScene01::UpdateCollectibles(float dt) {
	for (auto it = collectibles.begin(); it != collectibles.end(); ) {
		(*it)->Update(dt);

		if (!(*it)->IsActive()) {
			world->RemoveGameObject(*it);
			it = collectibles.erase(it);
		} else {
			++it;
		}
	}
}

