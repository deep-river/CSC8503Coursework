#pragma once
#include "GameObject.h"

namespace NCL {
	namespace CSC8503 {
		class PlayerObject : public GameObject {
		public:
			PlayerObject(const std::string& objectName = "Player");
			~PlayerObject();

			int AddScore(int score) {
				playerScore += score;
				return playerScore;
			}

			int GetScore() const {
				return playerScore;
			}

			void ResetScore() {
				playerScore = 0;
			}

		protected:
			int playerScore = 0; //Íæ¼Ò·ÖÊý
		};
	}
}


