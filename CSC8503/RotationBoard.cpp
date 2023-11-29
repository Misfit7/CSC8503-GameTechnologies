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

    physicsObject = new PhysicsObject(&transform, boundingVolume);
    physicsObject->SetInverseMass(inverseMass);
    physicsObject->InitCubeInertia();

    SetLockFlags(AxisLock::ANGULAR_X | AxisLock::ANGULAR_Z | AxisLock::LINEAR_X | AxisLock::LINEAR_Y | AxisLock::LINEAR_Z);
    SetUsesGravity(false);
}

void RotationBoard::update()
{
    Vector3 o = game.GetPlayer()->GetTransform().GetOrientation().ToEuler();
    Ray oRay = Ray(game.GetPlayer()->GetTransform().GetPosition(), o);

    RayCollision oCollision;
    oCollision.rayDistance = 10.0f;
    if (game.GetWorld()->Raycast(oRay, oCollision, true, game.GetPlayer())) {
        this->GetPhysicsObject()->AddForceAtPosition(game.GetPlayer()->GetTransform().GetOrientation().ToEuler() * 10, oCollision.collidedAt);
    }
}
