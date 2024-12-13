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
			
			void SetCaught(bool caught) {
				isCaught = caught;
			}

			bool IsCaught() const {
				return isCaught;
			}

		protected:
			int playerScore = 0; //玩家分数
			bool isCaught = false; //玩家是否被抓住
		};
	}
}


