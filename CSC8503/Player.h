#pragma once

#include "CourseWork.h"
#include "..\CSC8503CoreClasses\GameObject.h"
#include "..\CSC8503\GameTechRenderer.h"
#include "PositionConstraint.h"
#include "PlayerCamera.h"

namespace NCL {
    namespace CSC8503
    {
        class CourseWork;
        class PlayerCamera;

        class Player : public GameObject
        {
        public:
            Player(CourseWork& g, const Vector3& position,
                Mesh* mesh, Texture* basicTex, Shader* basicShader);

            void Update(float dt);

            void SetHealth(int h) { health = max(0, h); }
            int GetHealth() { return health; }
            int GetPower() { return power; }

            bool GetKey() { return getKey; }

            void SetIsHit(bool h) { isHit = h; }
            bool GetIsHit() { return isHit; }

            void OnCollisionBegin(GameObject* otherObject) override;

            void ResetKey();
            void ResetFinalTreasure();

            bool GetAttack() { return attack; }
        protected:
            CourseWork& game;
            GameWorld* world;

            void Respawn();

            float speed = 1.0f;
            float iMass;
            bool isStand;
            bool isGrab = false;
            bool isHit = false;
            float invincible = 0.0f;
            bool switchOrientation = false;
            int jumpCount = 0;
            int power = 0;
            bool attack = false;

            Vector3 grabPoint;

            //pathfind
            NavigationPath outPath;
            vector<Vector3> pathNodes;
            void Pathfinding();
            void DisplayPathfinding();

            bool getKey = false;
            //bool getFinalTreasure = false;

            PositionConstraint* constraint = nullptr;
        private:
            static int playerNum;
            int health = 2;
        };
    }
}
