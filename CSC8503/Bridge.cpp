#include "Bridge.h"

#include "..\CSC8503CoreClasses\PhysicsObject.h"
#include "..\CSC8503CoreClasses\RenderObject.h"
#include "..\NCLCoreClasses\Quaternion.h"
#include "PositionConstraint.h"

using namespace NCL;
using namespace CSC8503;

Bridge::Bridge(CourseWork& g, const Vector3& position,
    const Vector3& dimensions, int linkNum, float inverseMass) :game(g), world(g.GetWorld())
{
    if (linkNum < 2) return;
    Vector3 boardSize = dimensions;

    float maxDistance = 2.75 * dimensions.x; // constraint distance
    float cubeDistance = 2 * dimensions.x; // distance between links

    GameObject* linkStart = game.AddCubeToWorld(position, boardSize, 0.0f);
    GameObject* previous = linkStart;

    for (int i = 2; i <= linkNum - 1; ++i) {
        GameObject* sphere = game.AddCubeToWorld(position + Vector3(i * cubeDistance, 0.0f, 0.0f), boardSize, inverseMass);
        PositionConstraint* constraint = new PositionConstraint(previous, sphere, maxDistance);
        world->AddConstraint(constraint);
        previous = sphere;
    }

    GameObject* linkEnd = game.AddCubeToWorld(position + Vector3(linkNum * cubeDistance, 0.0f, 0.0f), boardSize, 0.0f);
    PositionConstraint* constraint = new PositionConstraint(previous, linkEnd, maxDistance);
    world->AddConstraint(constraint);
}

void Bridge::OnCollisionBegin(GameObject* otherObject)
{

}
