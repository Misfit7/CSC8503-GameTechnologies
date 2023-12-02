#include "PlayerCamera.h"
#include "Window.h"
#include "Maths.h"

using namespace NCL::CSC8503;
using namespace NCL::Maths;

void PlayerCamera::UpdateCamera(float dt)
{
    p -= (Window::GetMouse()->GetRelativePosition().y);

    y -= (Window::GetMouse()->GetRelativePosition().x);
    y = fmod(y, 360.0f);

    Matrix4 yawMat = Matrix4::Rotation(y, Vector3(0, yAixs, 0));
    Matrix4 pitchMat = Matrix4::Rotation(p, yawMat * Vector3(xAixs, 0, 0));
    Matrix4 finalRotMat = pitchMat * yawMat;
    //cout << finalRotMat;

    Vector3 focusPoint = player.GetTransform().GetPosition();
    Vector3 lookDirection = finalRotMat * Vector3(0, 0, -1);
    position = focusPoint - lookDirection * camDistance;
    //cout << position;

    Ray viewRay = Ray(focusPoint, -lookDirection);
    RayCollision rayData;
    if (world.Raycast(viewRay, rayData, true, &player))
        if (rayData.rayDistance < camDistance)
            position = focusPoint - lookDirection * (rayData.rayDistance - 1.0f);

    Matrix4 viewMatrix = Matrix4::BuildViewMatrix(position, player.GetTransform().GetPosition(), Vector3(0, yAixs, 0)).Inverse();

    Quaternion q(viewMatrix);
    //if (finalRotMat.GetDiagonal().x > -0.9999f)
    pitch = q.ToEuler().x;
    //else
    //    pitch = pitch;
    //cout << pitch << endl;
    yaw = q.ToEuler().y;

}

Matrix4 PlayerCamera::BuildProjectionMatrix(float currentAspect) const
{
    return Matrix4::Perspective(nearPlane, farPlane, currentAspect, fov);
}