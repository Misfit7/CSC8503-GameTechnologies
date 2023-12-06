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