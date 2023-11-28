#include "PlayerCamera.h"
#include "Window.h"
#include "Maths.h"

using namespace NCL::CSC8503;
using namespace NCL::Maths;

PlayerCamera::PlayerCamera(GameWorld& w, GameObject& o) : world(w), followTarget(o)
{
}

void PlayerCamera::UpdateCamera(float dt)
{
    v -= (Window::GetMouse()->GetRelativePosition().y);
    v = Clamp(v, -60.0f, 60.0f);

    h -= (Window::GetMouse()->GetRelativePosition().x);
    if (h < 0) h += 360.0f;
    if (h > 360.0f) h -= 360.0f;

    Matrix4 yawMat = Matrix4::Rotation(h, Vector3(0, 1, 0));
    Matrix4 pitchMat = Matrix4::Rotation(v, yawMat * Vector3(-1, 0, 0));
    Matrix4 finalRotMat = pitchMat * yawMat;

    Vector3 focusPoint = followTarget.GetTransform().GetPosition();
    Vector3 lookDirection = finalRotMat * Vector3(0, 0, -1);
    Vector3 upDirection = finalRotMat * Vector3(0, -1, 0);
    position = focusPoint - lookDirection * distanceOffset;

    Ray collisionRay = Ray(focusPoint, -lookDirection);
    RayCollision collisionRayData;
    if (world.Raycast(collisionRay, collisionRayData, true, &followTarget))
    {
        if (collisionRayData.rayDistance < distanceOffset)
            position = focusPoint - lookDirection * (collisionRayData.rayDistance - 1.0f);
    }

    Matrix4 viewMatrix = Matrix4::BuildViewMatrix(position, followTarget.GetTransform().GetPosition(), Vector3(0, 1, 0)).Inverse();
    Quaternion q(viewMatrix);
    pitch = q.ToEuler().x + pitchOffset;
    yaw = q.ToEuler().y;

    Matrix4 rotation = Matrix4::Rotation(yaw, Vector3(0, 1, 0)) * Matrix4::Rotation(pitch, Vector3(1, 0, 0));
    camX = rotation * Vector3(activeController->GetNamedAxis("Sidestep"), 0, 0);
    camY = rotation * Vector3(0, activeController->GetNamedAxis("UpDown"), 0);
    camZ = rotation * Vector3(0, 0, -activeController->GetNamedAxis("Forward"));
}

Matrix4 NCL::CSC8503::PlayerCamera::BuildProjectionMatrix(float currentAspect) const
{
    return Matrix4::Perspective(nearPlane, farPlane, currentAspect, fov);
}
