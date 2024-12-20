﻿#pragma once
//#include "TutorialGame.h"
#include "GameScene01.h"
#include "NetworkBase.h"

namespace NCL {
	namespace CSC8503 {
		class GameServer;
		class GameClient;
		class NetworkPlayer;
		class FullPacket;
		class DeltaPacket;
		class ClientPacket;
		//class MessagePacket;

		enum PlayerNetworkMessages {
			None,
			NEW_PLAYER_CONNECTED,
			PLAYER_DISCONNECTED,
		};

		struct MessagePacket : public GamePacket {
			short playerID;
			short messageID = PlayerNetworkMessages::None;

			MessagePacket() {
				type = Message;
				size = sizeof(short) * 2;
			}
		};

		//class NetworkedGame : public TutorialGame, public PacketReceiver {
		class NetworkedGame : public GameScene01, public PacketReceiver {
		public:
			NetworkedGame();
			~NetworkedGame();

			void StartAsServer();
			void StartAsClient(char a, char b, char c, char d);

			void UpdateGame(float dt) override;

			void SpawnPlayer();

			void StartNetLevel();

			void ReceivePacket(int type, GamePacket* payload, int source) override;

			void OnPlayerCollision(NetworkPlayer* a, NetworkPlayer* b);

			void InitNetworkedGameScene();

			void UpdateNetworkSelectionUI(float dt);

			int GeneratePlayerID();

			PlayerObject* AddNetPlayerToWorld(const Vector3& position, int playerID);



		protected:
			void UpdateAsServer(float dt);
			void UpdateAsClient(float dt);

			void BroadcastSnapshot(bool deltaFrame);
			void UpdateMinimumState();
			std::map<int, int> stateIDs;

			GameServer* thisServer;
			GameClient* thisClient;
			float timeToNextPacket;
			int packetsToSnapshot;

			std::vector<NetworkObject*> networkObjects;

			std::map<int, GameObject*> serverPlayers;

			bool isNetworkedGameStarted = false;

			int PlayerIDIndex = 0;
			int GlobalStateID;

			void OnReceiveFullState(FullPacket* packet);
			void OnReceiveDeltaState(DeltaPacket* packet);
			void OnReceivePlayerConnected(MessagePacket* packet);
			void OnReceivePlayerDisconnected(MessagePacket* packet);
			void OnReceiveMessage(MessagePacket* packet);
		};
	}
}

