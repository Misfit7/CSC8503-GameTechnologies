#pragma once

#include "CourseWork.h"
#include "..\CSC8503CoreClasses\GameObject.h"
#include "..\CSC8503\GameTechRenderer.h"

namespace NCL {
    namespace CSC8503
    {
        class CourseWork;

        class SpringBoard : public GameObject
        {
        public:
            SpringBoard(CourseWork& g, const Vector3& position, const Vector3& rotation, const Vector3& boardSize,
                Mesh* mesh, Texture* basicTex, Shader* basicShader, float inverseMass = 0.0f);

            void OnCollisionBegin(GameObject* otherObject) override;

        protected:
            CourseWork& game;

        };
    }
}