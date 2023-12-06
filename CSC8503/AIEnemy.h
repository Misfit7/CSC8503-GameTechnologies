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

        class AIEnemy :public GameObject
        {
        public:
            AIEnemy(CourseWork* g, const Vector3& position, Mesh* mesh, Texture* basicTex,
                Shader* basicShader, float inverseMass);
            void SetKeysPos(vector<Vector3> keysPos) {
                for (auto& i : keysPos) keysFound.emplace_back(pair<Vector3, bool>(i, false));
            };

        protected:
            CourseWork* game;
            GameWorld* world;
            NavigationGrid* grid;

            Vector3 originalPos;

            void Pathfinding(Vector3 startPos, Vector3 endPos);
            void DisplayPathfinding();

            //pathfind
            NavigationPath outPath;
            vector<Vector3> pathNodes;
            vector<pair<Vector3, bool>> keysFound;
            int currentKeyT = 0;
            int currentNodeT = 0;
        };
    }
}