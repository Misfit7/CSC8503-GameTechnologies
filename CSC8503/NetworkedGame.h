#pragma once
#include "CourseWork.h"
#include "NetworkBase.h"

namespace NCL {
    namespace CSC8503 {
        class GameServer;
        class GameClient;
        class NetworkPlayer;

        class NetworkedGame : public CourseWork, public PacketReceiver {
        public:
            NetworkedGame();
            ~NetworkedGame();

            void StartAsServer();
            void StartAsClient(char a, char b, char c, char d);

            void UpdateGame(float dt) override;

            void ReceivePacket(int type, GamePacket* payload, int source) override;

            void OnPlayerCollision(NetworkPlayer* a, NetworkPlayer* b);

            void SetGameState(int value);

        protected:
            void Menu(const std::string& text = "", const Vector4& colour = Debug::WHITE);

            void UpdateAsServer(float dt);
            void UpdateAsClient(float dt);

            Player* SpawnPlayer(int playerNum);

            void BroadcastSnapshot(bool deltaFrame);
            void UpdateMinimumState();
            std::map<int, int> stateIDs;

            GameServer* thisServer;
            GameClient* thisClient;
            float timeToNextPacket;
            int packetsToSnapshot;

            std::vector<NetworkObject*> networkObjects;

            std::map<int, GameObject*> serverPlayers;
            GameObject* localPlayer;

            string name;
        };

        class MainMenu : public PushdownState {
        public:
            MainMenu(NetworkedGame* g) {
                this->game = g;
            }
            ~MainMenu() {
            }
            PushdownResult OnUpdate(float dt, PushdownState** newState) override {
                if (game->GetCurrentGame() != 0) {
                    minChoice = 0;
                    maxChoice = 2;
                    Debug::Print("Resume", Vector2(45.0f, 20.0f), (currentChoice == 0) ? Debug::BLACK : Debug::WHITE);
                    Debug::Print("Restart Game", Vector2(40.0f, 40.0f), (currentChoice == 1) ? Debug::BLACK : Debug::WHITE);
                    Debug::Print("Quit", Vector2(47.0f, 60.0f), (currentChoice == 2) ? Debug::BLACK : Debug::WHITE);
                }
                else {
                    minChoice = 1;
                    maxChoice = 3;
                    if (game->gameTime < 0.0f)
                        Debug::Print("You Lose", Vector2(44.0f, 15.0f), Debug::RED);
                    else if (game->gameTime > 0.0f)
                        Debug::Print("You Won", Vector2(44.0f, 15.0f), Debug::BLUE);
                    Debug::Print("HostPlayer Game", Vector2(37.0f, 30.0f), (currentChoice == 1) ? Debug::BLACK : Debug::WHITE);
                    Debug::Print("MultiPlayer Game", Vector2(36.0f, 50.0f), (currentChoice == 2) ? Debug::BLACK : Debug::WHITE);
                    Debug::Print("Quit", Vector2(47.0f, 70.0f), (currentChoice == 3) ? Debug::BLACK : Debug::WHITE);
                }

                if (Window::GetKeyboard()->KeyPressed(KeyCodes::ESCAPE) && game->GetCurrentGame() != 0) {
                    Window::GetWindow()->ShowOSPointer(false);
                    Window::GetWindow()->LockMouseToWindow(true);
                    return PushdownResult::Pop;
                }
                if (Window::GetKeyboard()->KeyPressed(KeyCodes::UP)) {
                    currentChoice = std::max(currentChoice - 1, minChoice);
                }
                if (Window::GetKeyboard()->KeyPressed(KeyCodes::DOWN)) {
                    currentChoice = std::min(currentChoice + 1, maxChoice);
                }
                if (Window::GetKeyboard()->KeyPressed(KeyCodes::RETURN)) {
                    if (currentChoice == 0 && game->GetCurrentGame() != 0) {
                        Window::GetWindow()->ShowOSPointer(false);
                        Window::GetWindow()->LockMouseToWindow(true);
                        return PushdownResult::Pop;
                    }
                    else if (currentChoice == 1 && game->GetCurrentGame() != 0) {
                        Window::GetWindow()->ShowOSPointer(false);
                        Window::GetWindow()->LockMouseToWindow(true);
                        game->SetGameState(4);
                        return PushdownResult::Pop;
                    }
                    else if (currentChoice == 2 && game->GetCurrentGame() != 0) {
                        Window::GetWindow()->ShowOSPointer(false);
                        Window::GetWindow()->LockMouseToWindow(true);
                        game->SetGameState(3);
                        return PushdownResult::Pop;
                    }
                    Window::GetWindow()->ShowOSPointer(false);
                    Window::GetWindow()->LockMouseToWindow(true);
                    game->SetGameState(currentChoice);
                    return PushdownResult::Pop;
                }
                return PushdownResult::NoChange;
            };

            void OnAwake() override {
                currentChoice = game->GetCurrentGame() ? 0 : 1;
            }

        private:
            NetworkedGame* game;
            int currentChoice;
            int minChoice;
            int maxChoice;
        };
    }
}

