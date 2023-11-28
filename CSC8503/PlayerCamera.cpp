#include "PlayerCamera.h"
#include "Window.h"
#include "Maths.h"

using namespace NCL::CSC8503;
using namespace NCL::Maths;

void PlayerCamera::UpdateCamera(float dt)
{
    y -= (Window::GetMouse()->GetRelativePosition().y);
    y = Clamp(y, -60.0f, 89.0f);

    p -= (Window::GetMouse()->GetRelativePosition().x);
    if (p < 0) p += 360.0f;
    if (p > 360.0f) p -= 360.0f;

    Matrix4 yawMat = Matrix4::Rotation(p, Vector3(0, 1, 0));
    Matrix4 pitchMat = Matrix4::Rotation(y, yawMat * Vector3(-1, 0, 0));
    Matrix4 finalRotMat = pitchMat * yawMat;

    Vector3 focusPoint = player.GetTransform().GetPosition();
    Vector3 lookDirection = finalRotMat * Vector3(0, 0, -1);
    Vector3 upDirection = finalRotMat * Vector3(0, -1, 0);
    position = focusPoint - lookDirection * camDistance;

    Ray groundRay = Ray(focusPoint, -lookDirection);
    RayCollision rayData;
    if (world.Raycast(groundRay, rayData, true, &player))
    {
        if (rayData.rayDistance < camDistance)
            position = focusPoint - lookDirection * (rayData.rayDistance - 1.0f);
    }

    Matrix4 viewMatrix = Matrix4::BuildViewMatrix(position, player.GetTransform().GetPosition(), Vector3(0, 1, 0)).Inverse();
    Quaternion q(viewMatrix);
    pitch = q.ToEuler().x;
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