#include "DamageObject.h"
#include "..\CSC8503CoreClasses\PhysicsObject.h"
#include "..\CSC8503CoreClasses\RenderObject.h"
#include "..\NCLCoreClasses\Quaternion.h"
#include "PositionConstraint.h"

using namespace NCL;
using namespace CSC8503;

DamageObject::DamageObject(CourseWork& g, const Vector3& position,
    float radius, int linkNum, int impulseNum, float inverseMass) :game(g), world(g.GetWorld())
{
    if (linkNum < 2) return;
    if (impulseNum < 2 || impulseNum>linkNum) impulseNum = 2;
    float sphereSize = radius;

    float maxDistance = 3 * radius; // constraint distance
    float sphereDistance = 2 * radius; // distance between links

    GameObject* linkStart = game.AddSphereToWorld(position, sphereSize, 0.0f);
    GameObject* previous = linkStart;

    impulseObject = new GameObject;

    for (int i = 2; i <= linkNum; ++i) {
        GameObject* sphere = game.AddSphereToWorld(position + Vector3(0.0f, i * sphereDistance, 0.0f),
            sphereSize, inverseMass);
        PositionConstraint* constraint = new PositionConstraint(previous, sphere, maxDistance);
        world->AddConstraint(constraint);
        previous = sphere;
        if (i == impulseNum) impulseObject = sphere;
    }
    LinkMaxDistance = (linkNum - 1) * maxDistance;

}

void DamageObject::Update()
{
    if (impulseObject->GetTransform().GetPosition().y < (87.5 + LinkMaxDistance - 0.75f) && !bImpulse)
    {
        impulseObject->GetPhysicsObject()->ApplyLinearImpulse(Vector3(0.0f, 0.0f, -4.0f));
        bImpulse = true;
        //cout << bImpulse << endl;
    }
    if (impulseObject->GetTransform().GetPosition().y > 87.5 && bImpulse)
    {
        bImpulse = false;
        //cout << bImpulse << endl;
    }
}

void DamageObject::OnCollisionBegin(GameObject* otherObject)
{

}
