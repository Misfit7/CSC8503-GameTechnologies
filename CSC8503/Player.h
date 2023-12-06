#pragma once

#include "CourseWork.h"
#include "..\CSC8503CoreClasses\GameObject.h"
#include "..\CSC8503\GameTechRenderer.h"
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

            void SetHealth(int h) { health = min(0, h); }
            int GetHealth() { return health; }

            bool GetKey() { return getKey; }
        protected:
            CourseWork& game;
            GameWorld* world;

            void Respawn();

            float speed = 1.0f;
            float iMass;
            bool isStand;
            bool isGrab = false;
            bool switchOrientation = false;
            int jumpCount = 0;

            Vector3 grabPoint;

            //pathfind
            NavigationPath outPath;
            vector<Vector3> pathNodes;
            void Pathfinding();
            void DisplayPathfinding();

            bool getKey;
        private:
            static int playerNum;
            int health = 2;
        };
    }
}
