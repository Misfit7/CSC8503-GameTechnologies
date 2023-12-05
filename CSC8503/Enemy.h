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
            Enemy(CourseWork& g, const Vector3& position,
                Mesh* mesh, Texture* basicTex, Shader* basicShader);

            ~Enemy(void) { delete stateMachine; }
            void Update(float dt);

        protected:
            CourseWork& game;
            GameWorld* world;
            NavigationGrid* grid;

            float aliveTime = 0;
            StateMachine* stateMachine;

            void GenerateStateMachine();
            void Wander(float dt);
            //string moveDirection = "forward";
            void ChasePlayer(float dt);
            void Respawn();

            //pathfind
            NavigationPath outPath;
            vector<Vector3> pathNodes;
            void Pathfinding();
            void DisplayPathfinding();

        };

    }
}
