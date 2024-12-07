#include "GameScene01.h"
#include "GameWorld.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "TextureLoader.h"

#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "StateGameObject.h"



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

	forceMagnitude = 10.0f;
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

	InitialiseAssets();
	std::cout << ">>>>>>>>>>>>Scene: GameScene01 begin!<<<<<<<<<<<<" << std::endl;
}

void GameScene01::InitialiseAssets() {
	cubeMesh = renderer->LoadMesh("cube.msh");
	sphereMesh = renderer->LoadMesh("sphere.msh");
	catMesh = renderer->LoadMesh("ORIGAMI_Chat.msh");
	kittenMesh = renderer->LoadMesh("Kitten.msh");

	enemyMesh = renderer->LoadMesh("Keeper.msh");
	bonusMesh = renderer->LoadMesh("19463_Kitten_Head_v1.msh");
	capsuleMesh = renderer->LoadMesh("capsule.msh");

	basicTex = renderer->LoadTexture("checkerboard.png");
	basicShader = renderer->LoadShader("scene.vert", "scene.frag");

	InitWorld();
	InitCamera();
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

	delete player;

	delete physics;
	delete renderer;
	delete world;
}

void GameScene01::UpdateGame(float dt) {
	if (!isGameOver) {
		if (showMenu) {
			RenderMenu();
		}
		else {
			if (player != nullptr) {
				Vector3 objPos = player->GetTransform().GetPosition();
				Vector3 camPos = objPos + lockedOffset;

				Matrix4 temp = Matrix::View(camPos, objPos, Vector3(0, 1, 0));
				Matrix4 modelMat = Matrix::Inverse(temp);

				Quaternion q(modelMat);
				Vector3 angles = q.ToEuler(); //nearly there now!

				physics->UseGravity(useGravity);

				world->GetMainCamera().UpdateCamera(dt);
				world->GetMainCamera().SetPosition(camPos);
				//world->GetMainCamera().SetPitch(angles.x);
				//world->GetMainCamera().SetYaw(angles.y);
			}
			UpdateGameTimer(dt);
			UpdatePlayer(dt);
			UpdateGameUI();
			//This year we can draw debug textures as well!
			//Debug::DrawTex(*basicTex, Vector2(10, 10), Vector2(5, 5), Debug::MAGENTA);
			world->UpdateWorld(dt);
			physics->Update(dt);

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
	if (!player) {
		return;
	}
	float yaw = Window::GetMouse()->GetRelativePosition().x * 100.0f;
	player->GetTransform().SetOrientation(player->GetTransform().GetOrientation() * Quaternion::AxisAngleToQuaterion(Vector3(0, 1, 0), -yaw * dt));

	Quaternion objOrientation = player->GetTransform().GetOrientation();
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
		float currrentYSpeed = player->GetPhysicsObject()->GetLinearVelocity().y;
		player->GetPhysicsObject()->SetLinearVelocity(Vector3(0, currrentYSpeed, 0));
	} else if (Vector::Length(movement) > 0) {
		// Normalize and scale the movement
		Vector::Normalise(movement);
		movement *= playerMoveSpeed * dt;
		player->GetPhysicsObject()->ApplyLinearImpulse(movement);
	}
	//鼠标滚轮调整相机与角色之间的跟踪距离（目前只调整了Z轴距离）
	float moveWheelMovement = Window::GetMouse()->GetWheelMovement();
	cameraDistance += moveWheelMovement * 1.0f;
	cameraDistance = std::min(-5.0f, std::max(cameraDistance, -15.0f)); // Clamp between 5 and 20
	cameraHeight -= moveWheelMovement * 0.5f;
	cameraHeight = std::max(2.5f, std::min(cameraHeight, 7.5f));
	lockedOffset.y = cameraHeight;
	lockedOffset.z = cameraDistance;
	Vector3 objPos = player->GetTransform().GetPosition();
	Vector3 camPos = objPos + (objOrientation * lockedOffset);

	world->GetMainCamera().SetPosition(camPos);
	//world->GetMainCamera().SetPitch(0); //锁定摄像机Y轴移动
	world->GetMainCamera().SetYaw(player->GetTransform().GetOrientation().ToEuler().y + 180.0f);
}

void GameScene01::UpdateGameUI() {
	if (!isGameOver) {
		Debug::Print("GameScene01", Vector2(50, 5), Debug::WHITE);
		Debug::Print("Press (ESC) to toggle menu", Vector2(50, 10), Debug::WHITE);
		Debug::Print("Time left: " + std::to_string(static_cast<int>(gameTimer)), Vector2(2, 5), Debug::YELLOW);
		Debug::Print("Score: " + std::to_string(playerScore), Vector2(2, 10), Debug::YELLOW);
	}
}

void GameScene01::RenderMenu() {
	Debug::Print("Menu", Vector2(40, 40), Debug::WHITE);
	Debug::Print("1. Play Again", Vector2(40, 45), Debug::WHITE);
	Debug::Print("2. Exit Game", Vector2(40, 50), Debug::WHITE);

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::NUM1)) {
		// Reset game state
		gameTimer = gameDuration;
		playerScore = 0;
		isGameOver = false;
		showMenu = false;
		InitWorld();
	}
	else if (Window::GetKeyboard()->KeyPressed(KeyCodes::NUM2)) {
		exit(0);
	}
}

void GameScene01::RenderGameOverScreen() {
	Debug::Print("Game Over!", Vector2(40, 35), Debug::RED);
	Debug::Print("Final Score: " + std::to_string(playerScore), Vector2(40, 40), Debug::YELLOW);
	Debug::Print("1. Play Again", Vector2(40, 50), Debug::WHITE);
	Debug::Print("2. Exit Game", Vector2(40, 55), Debug::WHITE);

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::NUM1)) {
		gameTimer = gameDuration;
		playerScore = 0;
		isGameOver = false;
		showMenu = false;
		InitWorld();
	}
	else if (Window::GetKeyboard()->KeyPressed(KeyCodes::NUM2)) {
		exit(0);
	}
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
	InitTerrain(20, 20, 10.0f);
	InitGameExamples();
	//InitDefaultFloor();
}

GameObject* GameScene01::AddFloorToWorld(const Vector3& position) {
	GameObject* floor = new GameObject();

	Vector3 floorSize = Vector3(200, 2, 200);
	AABBVolume* volume = new AABBVolume(floorSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform()
		.SetScale(floorSize * 2.0f)
		.SetPosition(position);

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

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

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

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2.0f);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

GameObject* GameScene01::AddPlayerToWorld(const Vector3& position) {
	float meshSize = 1.0f;
	float inverseMass = 0.5f;

	GameObject* character = new GameObject();
	SphereVolume* volume = new SphereVolume(1.0f);

	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), catMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(character);

	return character;
}

GameObject* GameScene01::AddEnemyToWorld(const Vector3& position) {
	float meshSize = 3.0f;
	float inverseMass = 0.5f;

	GameObject* character = new GameObject();

	AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);
	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), enemyMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(character);

	return character;
}

GameObject* GameScene01::AddBonusToWorld(const Vector3& position) {
	GameObject* apple = new GameObject();

	SphereVolume* volume = new SphereVolume(0.5f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(2, 2, 2))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
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
		AddCubeToWorld(Vector3(-40 + i * blockSize, 5, -40), Vector3(blockSize / 2, 10, blockSize / 2), 0)->GetRenderObject()->SetColour(Vector4(0.5f, 0.5f, 0.5f, 1.0f));
		AddCubeToWorld(Vector3(-40 + i * blockSize, 5, 40), Vector3(blockSize / 2, 10, blockSize / 2), 0)->GetRenderObject()->SetColour(Vector4(0.5f, 0.5f, 0.5f, 1.0f));
	}
	for (int i = 0; i < 8; ++i) {
		AddCubeToWorld(Vector3(50, 5, -30 + i * blockSize), Vector3(blockSize / 2, 10, blockSize / 2), 0)->GetRenderObject()->SetColour(Vector4(0.5f, 0.5f, 0.5f, 1.0f));
	}

	// 2. Create some obstacles and decorative elements
	AddCubeToWorld(Vector3(0, 7.5, 0), Vector3(blockSize, 15, blockSize), 0)->GetRenderObject()->SetColour(Vector4(0.7f, 0.3f, 0.3f, 1.0f));
	AddCubeToWorld(Vector3(-30, 10, 0), Vector3(blockSize, 20, blockSize), 0)->GetRenderObject()->SetColour(Vector4(0.3f, 0.7f, 0.3f, 1.0f));
	AddCubeToWorld(Vector3(30, 12.5, 0), Vector3(blockSize, 25, blockSize), 0)->GetRenderObject()->SetColour(Vector4(0.3f, 0.3f, 0.7f, 1.0f));

	// 3. Create open areas for items
	// Left open area
	AddCubeToWorld(Vector3(-30, 2.5, -30), Vector3(blockSize * 1.5, 5, blockSize * 1.5), 0)->GetRenderObject()->SetColour(Vector4(0.7f, 0.7f, 0.3f, 1.0f));
	// Right open area
	AddCubeToWorld(Vector3(30, 2.5, 30), Vector3(blockSize * 1.5, 5, blockSize * 1.5), 0)->GetRenderObject()->SetColour(Vector4(0.3f, 0.7f, 0.7f, 1.0f));
}

void GameScene01::InitGameExamples() {
	player = AddPlayerToWorld(playerSpawnPos);
	player->GetRenderObject()->SetColour(playerColour);
	//AddEnemyToWorld(Vector3(5, 5, 0));
	//AddBonusToWorld(Vector3(10, 5, 0));
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

