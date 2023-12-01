#pragma once

#include "CourseWork.h"
#include "..\CSC8503CoreClasses\GameObject.h"
#include "..\CSC8503\GameTechRenderer.h"

namespace NCL {
    namespace CSC8503
    {
        class CourseWork;

        class Bridge :public GameObject
        {
        public:
            Bridge(CourseWork& g, const Vector3& position, const Vector3& dimensions, int linkNum,
                float inverseMass = 1.0f);

            void OnCollisionBegin(GameObject* otherObject);

        protected:
            CourseWork& game;
            GameWorld* world;

        };
    }
}