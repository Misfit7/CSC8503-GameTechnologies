#pragma once

#include "CourseWork.h"
#include "..\CSC8503CoreClasses\GameObject.h"
#include "..\CSC8503\GameTechRenderer.h"

namespace NCL {
    namespace CSC8503
    {
        class CourseWork;

        class DamageObject :public GameObject
        {
        public:
            DamageObject(CourseWork& g) :game(g) {}
            DamageObject(CourseWork& g, const Vector3& position, float radius, int linkNum, int impulseNum,
                float inverseMass, Mesh* mesh, Texture* tex, Shader* shader);

            void Update();
            void OnCollisionBegin(GameObject* otherObject);
            DamageObject* AddSphereToWorld(const Vector3& position, float radius, float inverseMass,
                Mesh* mesh, Texture* tex, Shader* shader);
            void SetImpulse(bool i) { bImpulse = i; }

        protected:
            CourseWork& game;
            GameWorld* world;
            GameObject* impulseObject;

            vector<GameObject*> spheres;

            float LinkMaxDistance;
            bool bImpulse = false;
        };
    }
}