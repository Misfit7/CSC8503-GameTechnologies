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

            void Update(float dt);
            void OnCollisionBegin(GameObject* otherObject) override;

        protected:
            CourseWork* game;
            GameWorld* world;
            NavigationGrid* grid;
            unordered_map<string, int> keysFound;

            Vector3 originalPos;
            Vector3 destinationPos;

            void Respawn();

            void Pathfinding(Vector3 startPos, Vector3 endPos);
            void DisplayPathfinding();
            void GenerateBehaviourTree();

            void Move(const Vector3& pos, float dt);
            void FindKeys(float dt);
            bool foundAllKeys;
            void GotoFinalTreasure(float dt);
            bool gotoSuccess;

            //pathfind
            NavigationPath outPath;
            vector<Vector3> pathNodes;

            int currentNodeT = 0;

            //behaviourtree
            BehaviourSequence* rootSequence;
            BehaviourSequence* sequence;
            BehaviourSelector* selection;
        };
    }
}