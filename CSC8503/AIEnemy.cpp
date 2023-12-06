#include "AIEnemy.h"

#include "..\CSC8503CoreClasses\PhysicsObject.h"
#include "..\CSC8503CoreClasses\RenderObject.h"
#include "..\NCLCoreClasses\Quaternion.h"
#include "StateGameObject.h"
#include "StateTransition.h"
#include "StateMachine.h"
#include "State.h"

using namespace std;
using namespace NCL;
using namespace CSC8503;

/*if (pathNodes.size())
    {
        float nodeDistance = (pathNodes[currentNodeT] - transform.GetPosition()).Length();
        if (nodeDistance <= 15)
        {
            currentNodeT++;
            if (currentNodeT >= (int)pathNodes.size() - 1)
                Pathfinding(transform.GetPosition(), keysFound[currentKeyT].first);
        }

        Move(pathNodes[currentNodeT++], dt);
    }
    currentNodeT = 0;*/

NCL::CSC8503::AIEnemy::AIEnemy(CourseWork* g, const Vector3& position, Mesh* mesh, Texture* basicTex,
    Shader* basicShader, float inverseMass) :game(g), world(g->GetWorld()), grid(g->GetGrid())
{
    originalPos = position;

    float meshSize = 1.75f;

    AABBVolume* volume = new AABBVolume(Vector3(1.0f, 1.0f, 1.0f) * meshSize);
    SetBoundingVolume((CollisionVolume*)volume);

    transform
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position)
        .SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 180.0f) *
            Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), 0.0f));

    renderObject = new RenderObject(&transform, mesh, nullptr, basicShader);
    renderObject->SetColour(Vector4(1, 0, 0, 1));

    physicsObject = new PhysicsObject(&transform, GetBoundingVolume());
    physicsObject->SetInverseMass(inverseMass);
    physicsObject->InitSphereInertia();
    //SetUsesGravity(false);

    world->AddGameObject(this);
}

void AIEnemy::Pathfinding(Vector3 startPos, Vector3 endPos) {

    outPath.Clear();
    pathNodes.clear();
    bool found = grid->FindPath(startPos, endPos, outPath);

    Vector3 pos;
    while (outPath.PopWaypoint(pos)) {
        pathNodes.push_back(pos);
    }

    DisplayPathfinding();
}

void AIEnemy::DisplayPathfinding() {
    for (int i = 1; i < pathNodes.size(); ++i) {
        Vector3 a = pathNodes[i - 1];
        Vector3 b = pathNodes[i];

        Debug::DrawLine(a, b, Vector4(0, 1, 0, 1));
    }
}