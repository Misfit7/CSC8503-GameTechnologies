#pragma once

#include "CourseWork.h"
#include "..\CSC8503CoreClasses\GameObject.h"
#include "..\CSC8503\GameTechRenderer.h"

namespace NCL {
    namespace CSC8503
    {
        class CourseWork;

        class Player : public GameObject
        {
        public:
            Player(CourseWork& g, const Vector3& position,
                Mesh* mesh, Texture* basicTex, Shader* basicShader);

            void Update(float dt);

        protected:
            CourseWork& game;
            GameWorld* world;

            bool isStand;
            int jumpCount = 0;
        };
    }
}