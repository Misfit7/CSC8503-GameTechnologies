//#pragma once
//#include "Camera.h"
//#include "..\CSC8503CoreClasses\Transform.h"
//#include "..\CSC8503\GameTechRenderer.h"
//
//namespace NCL::CSC8503
//{
//    class PlayerCamera : public Camera
//    {
//    public:
//        PlayerCamera(GameWorld& gWorld, GameObject& target);
//        ~PlayerCamera() {};
//
//        virtual void UpdateCamera(float dt) override;
//
//    protected:
//        GameWorld& world;
//
//        GameObject& followTarget;
//        Vector3 startOffset;
//        Vector3 followOffset = Vector3(0, 20.0f, 30.0f);
//
//        float pitchOffset = 12.0f;
//
//    private:
//        float requiredRayDistance;
//        float h = 0.0f, v = 0.0f;
//    };
//}