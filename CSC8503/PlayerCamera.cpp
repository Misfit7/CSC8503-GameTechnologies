//#include "PlayerCamera.h"
//#include "Window.h"
//#include "Maths.h"
//
//using namespace NCL::CSC8503;
//using namespace NCL::Maths;
//
//PlayerCamera::PlayerCamera(GameWorld& gWorld, GameObject& target) : world(gWorld), followTarget(target)
//{
//    requiredRayDistance = 36.0f;
//    startOffset = followOffset;
//}
//
//void PlayerCamera::UpdateCamera(float dt)
//{
//    if (!active) return;
//
//    v -= (Window::GetMouse()->GetRelativePosition().y);
//    v = Clamp(v, -45.0f, 45.0f);
//
//    h -= (Window::GetMouse()->GetRelativePosition().x);
//    if (h < 0) h += 360.0f;
//    if (h > 360.0f) h -= 360.0f;
//
//    Matrix4 yawMat = Matrix4::Rotation(h, Vector3(0, 1, 0));
//    Matrix4 pitchMat = Matrix4::Rotation(v, yawMat * Vector3(-1, 0, 0));
//    Matrix4 finalRotMat = pitchMat * yawMat;
//
//    Vector3 focusPoint = followTarget.GetTransform().GetPosition();
//    Vector3 lookDirection = finalRotMat * Vector3(0, 0, -1);
//
//    position = focusPoint - lookDirection * requiredRayDistance;
//
//    //Prevent Wall Clipping
//    Ray collisionRay = Ray(focusPoint, -lookDirection);
//    RayCollision collisionRayData;
//    if (world.Raycast(collisionRay, collisionRayData, true, &followTarget))
//    {
//        if (collisionRayData.rayDistance < requiredRayDistance)
//            position = focusPoint - lookDirection * (collisionRayData.rayDistance - 1.0f);
//    }
//
//    //Stare at GOAT!
//    Matrix4 viewMatrix = Matrix4::BuildViewMatrix(position, followTarget.GetTransform().GetPosition(), Vector3(0, 1, 0)).Inverse();
//    /*   Quaternion q(viewMatrix);
//       pitch = q.ToEuler().x + pitchOffset;
//       yaw = q.ToEuler().y;
//
//       Matrix4 rotation = Matrix4::Rotation(yaw, Vector3(0, 1, 0)) * Matrix4::Rotation(pitch, Vector3(1, 0, 0));
//       camForward = rotation * Vector3(0, 0, -1);
//       camRight = rotation * Vector3(1, 0, 0);
//       camUp = rotation * Vector3(0, 1, 0);*/
//}