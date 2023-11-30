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
            DamageObject(CourseWork& g, const Vector3& position, float radius, int linkNum, int impulseNum,
                float inverseMass = 1.0f);

            void Update();
            void OnCollisionBegin(GameObject* otherObject);

        protected:
            CourseWork& game;
            GameWorld* world;
            GameObject* impulseObject;
            float LinkMaxDistance;

        };
    }
}