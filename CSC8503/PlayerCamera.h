#pragma once
#include "Camera.h"
#include "..\CSC8503CoreClasses\Transform.h"
#include "..\CSC8503\GameTechRenderer.h"

namespace NCL::CSC8503
{
    class PlayerCamera : public PerspectiveCamera
    {
    public:
        PlayerCamera(GameWorld& w, GameObject& o);
        ~PlayerCamera() {};

        virtual void UpdateCamera(float dt) override;

    protected:
        GameWorld& world;
        GameObject& followTarget;

        Matrix4 BuildProjectionMatrix(float aspectRatio = 1.0f) const override;
    private:
        float pitchOffset = 0.0f;
        float distanceOffset = 10.0f;
        float h = 0.0f, v = 0.0f;
    };
}