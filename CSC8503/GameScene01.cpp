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
	inSelectionMode = false;

	world->GetMainCamera().SetController(controller);

	controller.MapAxis(0, "Sidestep");
	controller.MapAxis(1, "UpDown");
	controller.MapAxis(2, "Forward");

	controller.MapAxis(3, "XLook");
	controller.MapAxis(4, "YLook");

	InitialiseAssets();
	std::cout << ">>>>>>>>>>>>Scene: GameScene01 begin!<<<<<<<<<<<<" << std::endl;
}

/*

Each of the little demo scenarios used in the game uses the same 2 meshes,
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
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

	delete physics;
	delete renderer;
	delete world;
}

void GameScene01::UpdateGame(float dt) {
	if (!inSelectionMode) {
		world->GetMainCamera().UpdateCamera(dt);
	}
	if (lockedObject != nullptr) {
		Vector3 objPos = lockedObject->GetTransform().GetPosition();
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

	UpdatePlayer(dt);

	if (useGravity) {
		Debug::Print("(G)ravity on", Vector2(5, 95), Debug::RED);
	}
	else {
		Debug::Print("(G)ravity off", Vector2(5, 95), Debug::RED);
	}
	//This year we can draw debug textures as well!
	//Debug::DrawTex(*basicTex, Vector2(10, 10), Vector2(5, 5), Debug::MAGENTA);

	world->UpdateWorld(dt);
	renderer->Update(dt);
	physics->Update(dt);

	renderer->Render();
	Debug::UpdateRenderables(dt);
}

void GameScene01::UpdatePlayer(float dt) {
	if (!player || !lockedObject) {
		return;
	}

	float yaw = Window::GetMouse()->GetRelativePosition().x * 100.0f;
	//std::cout << "Yaw: " << yaw << std::endl;
	player->GetTransform().SetOrientation(player->GetTransform().GetOrientation() * Quaternion::AxisAngleToQuaterion(Vector3(0, 1, 0), -yaw * dt));

	Quaternion objOrientation = player->GetTransform().GetOrientation();
	Vector3 forward = objOrientation * Vector3(0, 0, -1);
	Vector3 right = objOrientation * Vector3(1, 0, 0);

	// Move the object based on keyboard input
	Vector3 movement(0, 0, 0);
	if (Window::GetKeyboard()->KeyDown(KeyCodes::W)) {
		movement += forward;
	}
	if (Window::GetKeyboard()->KeyDown(KeyCodes::S)) {
		movement -= forward;
	}
	if (Window::GetKeyboard()->KeyDown(KeyCodes::A)) {
		movement -= right;
	}
	if (Window::GetKeyboard()->KeyDown(KeyCodes::D)) {
		movement += right;
	}

	// Normalize and scale the movement
	if (Vector::Length(movement) > 0) {
		Vector::Normalise(movement);
		movement *= playerMoveSpeed * dt;
		player->GetPhysicsObject()->ApplyLinearImpulse(movement);
	}

	Vector3 objPos = player->GetTransform().GetPosition();
	Vector3 camPos = objPos + (objOrientation * lockedOffset);

	world->GetMainCamera().SetPosition(camPos);
	//world->GetMainCamera().SetPitch(0); //锁定摄像机Y轴移动
	world->GetMainCamera().SetYaw(player->GetTransform().GetOrientation().ToEuler().y);
}

void GameScene01::InitCamera() {
	world->GetMainCamera().SetNearPlane(0.1f);
	world->GetMainCamera().SetFarPlane(500.0f);
	world->GetMainCamera().SetPitch(-15.0f);
	world->GetMainCamera().SetYaw(315.0f);
	world->GetMainCamera().SetPosition(Vector3(-60, 40, 60));
	lockedObject = player; // camera look at
}

void GameScene01::InitWorld() {
	world->ClearAndErase();
	physics->Clear();

	//InitMixedGridWorld(15, 15, 3.5f, 3.5f);

	//BridgeConstraintTest(); //重力吊桥物理约束测试

	InitGameExamples();
	InitDefaultFloor();
}

/*

A single function to add a large immoveable cube to the bottom of our world

*/
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

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple'
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
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

void GameScene01::InitGameExamples() {
	player = AddPlayerToWorld(Vector3(0, 5, 0));
	AddEnemyToWorld(Vector3(5, 5, 0));
	AddBonusToWorld(Vector3(10, 5, 0));
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

