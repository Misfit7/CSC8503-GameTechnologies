#include "SpringBoard.h"
#include "..\CSC8503CoreClasses\PhysicsObject.h"
#include "..\CSC8503CoreClasses\RenderObject.h"
#include "..\NCLCoreClasses\Quaternion.h"

using namespace NCL;
using namespace CSC8503;

SpringBoard::SpringBoard(CourseWork& g, const Vector3& position, const Vector3& rotation, const Vector3& boardSize,
    Mesh* mesh, Texture* basicTex, Shader* basicShader, float inverseMass) :game(g)
{
    AABBVolume* volume = new AABBVolume(boardSize);
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
    SetUsesGravity(false);

    game.GetWorld()->AddGameObject(this);
}

void SpringBoard::OnCollisionBegin(GameObject* otherObject)
{
    if (otherObject == game.GetPlayer() && game.GetPlayer()->GetKey())
        otherObject->GetPhysicsObject()->ApplyLinearImpulse(transform.GetUp() * 25.0f);
    else if (otherObject == game.GetPlayer() || game.GetPlayer()->GetKey())
        otherObject->GetPhysicsObject()->ApplyLinearImpulse(transform.GetRight() * 100.0f);
}
