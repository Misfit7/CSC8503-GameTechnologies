#pragma once
#include <sstream>
#include <string>

#include "CourseWork.h"
#include "..\CSC8503CoreClasses\GameObject.h"
#include "..\CSC8503\GameTechRenderer.h"

#include "NavigationGrid.h"
#include "NavigationMesh.h"

namespace NCL {
    namespace CSC8503
    {
        class CourseWork;
        class StateMachine;
        class NavigationGrid;

        enum EnemyState
        {
            WANDER = 0,
            CHASE = 1,
            DEAD = 2,
        };

        class Enemy :public GameObject
        {
        public:
            Enemy(CourseWork* g, const Vector3& position,
                Mesh* mesh, Texture* basicTex, Shader* basicShader, float inverseMass, string t = "");

            ~Enemy(void) { delete stateMachine; }
            void Update(float dt);
            void OnCollisionBegin(GameObject* otherObject) override;

        protected:
            CourseWork* game;
            GameWorld* world;
            NavigationGrid* grid;

            Vector3 originalPos;

            Quaternion ogQ;
            bool switchOrientation = false;
            string type;

            float aliveTime = 0;
            StateMachine* stateMachine;

            void GenerateStateMachine();
            void Wander(float dt);
            string moveDirection = "";
            void ChasePlayer(float dt);
            void Respawn();

            void Move(const Vector3& pos, float dt);
        };

    }
}
