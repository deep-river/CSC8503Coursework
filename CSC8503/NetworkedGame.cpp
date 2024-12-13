#include "NetworkedGame.h"
#include "NetworkPlayer.h"
#include "NetworkObject.h"
#include "GameServer.h"
#include "GameClient.h"

#define COLLISION_MSG 30


NetworkedGame::NetworkedGame()	{
	thisServer = nullptr;
	thisClient = nullptr;

	NetworkBase::Initialise();
	timeToNextPacket  = 0.0f;
	packetsToSnapshot = 0;

	InitNetworkedGameScene();
	isNetworkedGameStarted = false;
	std::cout << ">>>>>>>>>>[NetworkedGame created!   ]<<<<<<<<<<" << std::endl;

}

//初始化场景，但不包括道具和玩家
//todo: 【有可能】道具和玩家可能需要分离，避免新玩家的加入初始化所有道具
NetworkedGame::~NetworkedGame()	{
	delete thisServer;
	delete thisClient;
}

//初始化联网场景
void NetworkedGame::InitNetworkedGameScene() {
	InitWorld();
	InitCamera();
}

void NetworkedGame::StartAsServer() {
	thisServer = new GameServer(NetworkBase::GetDefaultPort(), 4);

	thisServer->RegisterPacketHandler(Received_State, this);

	isNetworkedGameStarted = true;
	std::cout << ">>>>>>>>>>[NetworkedGame: Started as server!]<<<<<<<<<<" << std::endl;

	//StartNetLevel();

	InitGameObjects();
	GlobalStateID = -1;

	PlayerObject* player = AddNetPlayerToWorld(playerSpawnPos, GeneratePlayerID());
	localPlayer = player;

}

void NetworkedGame::StartAsClient(char a, char b, char c, char d) {
	thisClient = new GameClient();

	bool success = thisClient->Connect(a, b, c, d, NetworkBase::GetDefaultPort());
	if (!success) {
		std::cout << "Failed to connect to server!" << std::endl;
		return;
	}

	thisClient->RegisterPacketHandler(Delta_State, this);
	thisClient->RegisterPacketHandler(Full_State, this);
	thisClient->RegisterPacketHandler(Player_Connected, this);
	thisClient->RegisterPacketHandler(Player_Disconnected, this);

	isNetworkedGameStarted = true;
	std::cout << ">>>>>>>>>>[NetworkedGame: Started as client!]<<<<<<<<<<" << std::endl;

	StartNetLevel();
}

void NetworkedGame::UpdateGame(float dt) {
	timeToNextPacket -= dt;
	if (timeToNextPacket < 0) {
		if (thisServer) {
			UpdateAsServer(dt);
		}
		else if (thisClient) {
			UpdateAsClient(dt);
		}
		timeToNextPacket += 1.0f / 20.0f; //20hz server/client update
	}

	if (!isNetworkedGameStarted && !thisServer && Window::GetKeyboard()->KeyPressed(KeyCodes::F9)) {
		StartAsServer();
	}
	if (!isNetworkedGameStarted && !thisClient && Window::GetKeyboard()->KeyPressed(KeyCodes::F10)) {
		StartAsClient(127,0,0,1);
	}

	if (!isNetworkedGameStarted) {
		UpdateNetworkSelectionUI(dt);
	} else {
		GameScene01::UpdateGame(dt);
	}
}

//按F9键启动服务器，按F10键启动客户端
void NetworkedGame::UpdateNetworkSelectionUI(float dt) {
	Debug::Print("Game mode selection: ", Vector2(25, 30), Debug::WHITE);
	Debug::Print("Press F9 to start as server.", Vector2(25, 40), Debug::WHITE);
	Debug::Print("Press F10 to start as client.", Vector2(25, 45), Debug::WHITE);
	Debug::Print("Press Esc to quit game.", Vector2(25, 50), Debug::WHITE);

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::ESCAPE)) {
		exit(0);
	}

	//world->UpdateWorld(dt);
	renderer->Update(dt);
	renderer->Render();
	Debug::UpdateRenderables(dt);
}

void NetworkedGame::UpdateAsServer(float dt) {
	packetsToSnapshot--;
	if (packetsToSnapshot < 0) {
		BroadcastSnapshot(false);
		packetsToSnapshot = 5;
	}
	else {
		BroadcastSnapshot(true);
	}
	UpdateMinimumState();
}

void NetworkedGame::UpdateAsClient(float dt) {
	ClientPacket newPacket;

	if (Window::GetKeyboard()->KeyPressed(KeyCodes::SPACE)) {
		//fire button pressed!
		newPacket.buttonstates[0] = 1;
		newPacket.lastID = 0; //You'll need to work this out somehow...
	}
	thisClient->SendPacket(newPacket);
}

void NetworkedGame::BroadcastSnapshot(bool deltaFrame) {
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;

	world->GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		NetworkObject* o = (*i)->GetNetworkObject();
		if (!o) {
			continue;
		}
		//TODO - you'll need some way of determining
		//when a player has sent the server an acknowledgement
		//and store the lastID somewhere. A map between player
		//and an int could work, or it could be part of a 
		//NetworkPlayer struct. 
		int playerState = 0;
		GamePacket* newPacket = nullptr;
		if (o->WritePacket(&newPacket, deltaFrame, playerState)) {
			thisServer->SendGlobalPacket(*newPacket);
			delete newPacket;
		}
	}
}

void NetworkedGame::UpdateMinimumState() {
	//Periodically remove old data from the server
	int minID = INT_MAX;
	int maxID = 0; //we could use this to see if a player is lagging behind?

	for (auto i : stateIDs) {
		minID = std::min(minID, i.second);
		maxID = std::max(maxID, i.second);
	}
	//every client has acknowledged reaching at least state minID
	//so we can get rid of any old states!
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	world->GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		NetworkObject* o = (*i)->GetNetworkObject();
		if (!o) {
			continue;
		}
		o->UpdateStateHistory(minID); //clear out old states so they arent taking up memory...
	}
}

void NetworkedGame::StartNetLevel() {
	InitGameObjects();
	SpawnPlayer();
	GlobalStateID = -1;
}

void NetworkedGame::SpawnPlayer() {
	PlayerObject* player = AddNetPlayerToWorld(playerSpawnPos, GeneratePlayerID());
	localPlayer = player;
}

PlayerObject* NetworkedGame::AddNetPlayerToWorld(const Vector3& position, int playerID) {
	PlayerObject* player = AddPlayerToWorld(position);

	NetworkObject* netObj = new NetworkObject(*player, playerID);
	player->SetNetworkObject(netObj);
	serverPlayers[playerID] = player;
	std::cout << "Player " << playerID << " spawned!" << std::endl;

	return player;
}

void NetworkedGame::OnReceiveFullState(FullPacket* packet) {
	auto itr = serverPlayers.find(packet->objectID);
	if (itr == serverPlayers.end()) {
		std::cout << "Client Num " << " can't find netObject" << std::endl;
		return;
	}
	serverPlayers[packet->objectID]->GetNetworkObject()->ReadPacket(*packet);
	
	if (packet->fullState.stateID > GlobalStateID) { 
		GlobalStateID = packet->fullState.stateID; 
	}
}

void NetworkedGame::OnReceiveDeltaState(DeltaPacket* packet) {
	auto itr = serverPlayers.find(packet->objectID);
	if (itr == serverPlayers.end()) {
		std::cout << "Client Num " << " can't find netObject" << std::endl;
		return;
	}
	serverPlayers[packet->objectID]->GetNetworkObject()->ReadPacket(*packet);
}

void NetworkedGame::OnReceivePlayerConnected(MessagePacket* packet) {
	int newPlayerID = GeneratePlayerID();

	PlayerObject* newPlayer = AddNetPlayerToWorld(playerSpawnPos, newPlayerID);

	if (newPlayer) {
		// Notify all clients about the new player
		MessagePacket newPlayerPacket;
		newPlayerPacket.messageID = NEW_PLAYER_CONNECTED;
		newPlayerPacket.playerID = newPlayerID;
		thisServer->SendGlobalPacket(newPlayerPacket);

		std::cout << "Player " << newPlayerID << " connected and spawned at position: "
			<< playerSpawnPos.x << ", " << playerSpawnPos.y << std::endl;
	}
	else {
		std::cout << "Failed to spawn player " << newPlayerID << std::endl;
	}
}

void NetworkedGame::OnReceivePlayerDisconnected(MessagePacket* packet) {

}

void NetworkedGame::OnReceiveMessage(MessagePacket* packet) {
	
}

void NetworkedGame::ReceivePacket(int type, GamePacket* payload, int source) {
	switch (type) {
    case Full_State:
        if (auto* packet = static_cast<FullPacket*>(payload)) {
            OnReceiveFullState(packet);
        }
        break;
    case Delta_State:
        if (auto* packet = static_cast<DeltaPacket*>(payload)) {
            OnReceiveDeltaState(packet);
        }
        break;
    case Player_Connected:
        if (auto* packet = static_cast<MessagePacket*>(payload)) {
            OnReceivePlayerConnected(packet);
        }
        break;
    case Player_Disconnected:
        if (auto* packet = static_cast<MessagePacket*>(payload)) {
            OnReceivePlayerDisconnected(packet);
        }
        break;
    case Message:
        if (auto* packet = static_cast<MessagePacket*>(payload)) {
            OnReceiveMessage(packet);
        }
        break;
    default:
        std::cout << "Received unknown packet type: " << type << std::endl;
    }
}

void NetworkedGame::OnPlayerCollision(NetworkPlayer* a, NetworkPlayer* b) {
	if (thisServer) { //detected a collision between players!
		MessagePacket newPacket;
		newPacket.messageID = COLLISION_MSG;
		newPacket.playerID  = a->GetPlayerNum();

		thisClient->SendPacket(newPacket);

		newPacket.playerID = b->GetPlayerNum();
		thisClient->SendPacket(newPacket);
	}
}

int NetworkedGame::GeneratePlayerID() {
	return ++PlayerIDIndex;
}