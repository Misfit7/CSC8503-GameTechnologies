#include "RotationBoard.h"

#include "..\CSC8503CoreClasses\PhysicsObject.h"
#include "..\CSC8503CoreClasses\RenderObject.h"
#include "..\NCLCoreClasses\Quaternion.h"

using namespace NCL;
using namespace CSC8503;

RotationBoard::RotationBoard(CourseWork& g, const Vector3& position, const Vector3& rotation, const Vector3& boardSize,
    Mesh* mesh, Texture* basicTex, Shader* basicShader, float inverseMass) :game(g)
{
    OBBVolume* volume = new OBBVolume(boardSize);
    SetBoundingVolume((CollisionVolume*)volume);

    transform
        .SetScale(boardSize * 2)
        .SetPosition(position)
        .SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(1.0f, 0.0f, 0.0f), rotation.x) *
            Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), rotation.y) *
            Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), rotation.z));

    renderObject = new RenderObject(&transform, mesh, basicTex, basicShader);
    renderObject->SetColour(Vector4(0.0, 0, 0.7, 1));

    physicsObject = new PhysicsObject(&transform, GetBoundingVolume());
    physicsObject->SetInverseMass(inverseMass);
    physicsObject->InitCubeInertia();

    SetLockFlags(AxisLock::ANGULAR_X | AxisLock::ANGULAR_Z | AxisLock::LINEAR_X | AxisLock::LINEAR_Y | AxisLock::LINEAR_Z);

    game.GetWorld()->AddGameObject(this);
}

void RotationBoard::update()
{
    physicsObject->AddTorque(Vector3(0, -90.0f / physicsObject->GetInverseMass(), 0));
}
