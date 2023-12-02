#pragma once
#include "Camera.h"
#include "..\CSC8503CoreClasses\Transform.h"
#include "..\CSC8503\GameTechRenderer.h"

namespace NCL::CSC8503
{
    class PlayerCamera : public PerspectiveCamera
    {
    public:
        PlayerCamera(GameWorld& w, GameObject& p) : world(w), player(p) {};
        ~PlayerCamera() {};

        virtual void UpdateCamera(float dt) override;

        void SetDU(float d, float u) { down = d; up = u; }
        void SetViewMat(float x, float y) { xAixs = x; yAixs = y; }

    protected:
        GameWorld& world;
        GameObject& player;
        Matrix4 BuildProjectionMatrix(float aspectRatio = 1.0f) const override;

        Matrix4 pitchMat;

        float xAixs;
        float yAixs;
        float down, up;

        float camDistance = 7.5f;
        float p = 0.0f;
        float y = 0.0f;
    };
}