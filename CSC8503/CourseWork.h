#include "../NCLCoreClasses/KeyboardMouseController.h"

#pragma once
#include "GameTechRenderer.h"
#ifdef USEVULKAN
#include "GameTechVulkanRenderer.h"
#endif
#include "PhysicsSystem.h"

#include "StateGameObject.h"

#include "NavigationGrid.h"

#include "PushdownMachine.h"
#include "PushdownState.h"
#include "BehaviourAction.h"
#include "BehaviourSequence.h"
#include "BehaviourSelector.h"

#include "Player.h"
#include "Enemy.h"
#include "AIEnemy.h"
#include "PlayerCamera.h"
#include "SpringBoard.h"
#include "RotationBoard.h"
#include "DamageObject.h"
#include "Bridge.h"

#include <vector>

using namespace std;

namespace NCL {
    namespace CSC8503 {
        class Player;
        class Enemy;
        class AIEnemy;
        class PlayerCamera;
        class SpringBoard;
        class RotationBoard;
        class DamageObject;
        class Bridge;

        class CourseWork {
        public:
            CourseWork();
            ~CourseWork();

            virtual void UpdateGame(float dt);
            int GetCurrentGame() { return currentGame; }
            void SetGameState(int value);
            PlayerCamera* GetPlayerCamera() { return playerCamera; }
            bool GetSwitchCamera() { return switchCamera; }

            void SetSpringFlag(bool s) { springFlag = s; }

            GameObject* GetFinalTreasure() { return finalTreasure; }
            Vector3 GetFinalTreasurePos() { return finalTreasurePos; }

            Player* GetPlayer() { return player; }
            AIEnemy* GetAIEnemy() { return aiEnemy; }
            GameWorld* GetWorld() { return world; }
            NavigationGrid* GetGrid() { return grid; }
            vector<Vector3> GetKeysPos() { return keysPos; }
            GameObject* GetKey() { return key; }
            unordered_map<string, int>& GetKeysFound() { return keysFound; };
            vector<Vector3> GetSafePos() { return safePos; }

            GameObject* AddSphereToWorld(const Vector3& position, float radius, float inverseMass = 10.0f);
            GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);

            float gameTime = 0.0f;
        protected:
            //Game One
            Window* window;
            void Menu(const std::string& text = "", const Vector4& colour = Debug::WHITE);
            void ShowUIOne();
            void ShowUITwo();
            int currentGame = 0;
            void GameOneRunning(float dt);
            void GameTwoRunning(float dt);

            void InitialiseAssets();

            void InitMainCamera();
            void InitPlayerCamera();
            PlayerCamera* playerCamera;
            PerspectiveCamera* mainCamera;
            bool switchCamera = false;

            Player* player;
            AIEnemy* aiEnemy;
            vector<Enemy*> enemies;
            DamageObject* DamageLinkSphere;
            SpringBoard* springBoard;
            vector<RotationBoard*> rotationBoard;
            Bridge* bridge;
            GameObject* key;
            vector<GameObject*> keys;
            vector<Vector3> keysPos;
            //vector<pair<Vector3, int>> keysFound;
            unordered_map<string, int> keysFound;
            Vector3 finalTreasurePos;
            vector<Vector3> safePos;
            GameObject* finalTreasure;

            bool springFlag = false;
            float nodeSize;

            /*void InitKeysFound(vector<Vector3> keysPos) {
                for (auto& i : keysPos) keysFound.emplace_back(pair<Vector3, int>(i, 0));
            };*/
            void InitKeysFound(vector<Vector3> keysPos) {
                for (auto& i : keysPos)
                {
                    string s = to_string(i.x) + "/" + to_string(i.y) + "/" + to_string(i.z);
                    keysFound[s] = 0;
                }
            };

            void UpdateKeys();

            void InitGameOne();

            /*
            These are some of the world/object creation functions I created when testing the functionality
            in the module. Feel free to mess around with them to see different objects being created in different
            test scenarios (constraints, collision types, and so on).
            */
            void InitFloor();

            void InitGameOneObject();

            bool SelectObject();
            void MoveSelectedObject();
            void DebugObjectMovement();

            GameObject* AddFloorToWorld(const Vector3& position, float fSize);
            GameObject* AddBoardToWorld(const Vector3& position, const Vector3& rotation, const Vector3& boardSize, float inverseMass = 0.0f);

            GameObject* AddCapsuleToWorld(const Vector3& position, float meshSize = 1.0f, const std::string& name = "", float inverseMass = 0.0f);
#ifdef USEVULKAN
            GameTechVulkanRenderer* renderer;
#else
            GameTechRenderer* renderer;
#endif
            PhysicsSystem* physics;
            GameWorld* world;

            KeyboardMouseController controller;

            bool useGravity = true;
            bool inSelectionMode;

            float		forceMagnitude;

            GameObject* selectionObject = nullptr;

            Mesh* capsuleMesh = nullptr;
            Mesh* cubeMesh = nullptr;
            Mesh* sphereMesh = nullptr;

            Texture* basicTex = nullptr;
            Shader* basicShader = nullptr;

            //Coursework Meshes
            Mesh* charMesh = nullptr;
            Mesh* enemyMesh = nullptr;
            Mesh* AIenemyMesh = nullptr;

            //Coursework Additional functionality	

            GameObject* objClosest = nullptr;

            //Map
            NavigationGrid* grid;
        };
    }

    class MainMenu : public PushdownState {
    public:
        MainMenu(CourseWork* g) {
            this->game = g;
        }
        ~MainMenu() {
        }
        PushdownResult OnUpdate(float dt, PushdownState** newState) override {
            if (game->GetCurrentGame() != 0) {
                minChoice = 0;
                maxChoice = 3;
                Debug::Print("Resume", Vector2(45.0f, 20.0f), (currentChoice == 0) ? Debug::BLACK : Debug::WHITE);
                Debug::Print("Restart Game One", Vector2(36.0f, 40.0f), (currentChoice == 1) ? Debug::BLACK : Debug::WHITE);
                Debug::Print("Restart Game Two", Vector2(36.0f, 60.0f), (currentChoice == 2) ? Debug::BLACK : Debug::WHITE);
                Debug::Print("Quit", Vector2(47.0f, 80.0f), (currentChoice == 3) ? Debug::BLACK : Debug::WHITE);
            }
            else {
                minChoice = 1;
                maxChoice = 3;
                if (game->gameTime < 0.0f)
                    Debug::Print("You Lose", Vector2(44.0f, 15.0f), Debug::RED);
                else if (game->gameTime > 0.0f)
                    Debug::Print("You Won", Vector2(44.0f, 15.0f), Debug::BLUE);
                Debug::Print("SinglePlayer Game", Vector2(35.0f, 30.0f), (currentChoice == 1) ? Debug::BLACK : Debug::WHITE);
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
        CourseWork* game;
        int currentChoice;
        int minChoice;
        int maxChoice;
    };
}

