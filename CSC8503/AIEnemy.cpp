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

AIEnemy::AIEnemy(CourseWork* g, const Vector3& position, Mesh* mesh, Texture* basicTex,
    Shader* basicShader, float inverseMass) :game(g), world(g->GetWorld()), grid(g->GetGrid())
{
    foundAllKeys = false;
    gotoSuccess = false;
    originalPos = position;

    float meshSize = 0.75f;

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

    GenerateBehaviourTree();
}

void AIEnemy::Pathfinding(Vector3 startPos, Vector3 endPos) {
    startPos.x += 2.5f;
    startPos.z += 2.5f;

    outPath.Clear();
    pathNodes.clear();
    grid->FindPath(startPos, endPos, outPath);

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

        Debug::DrawLine(a, b, Vector4(1, 0, 0, 1));
    }
}

void AIEnemy::Move(const Vector3& pos, float dt)
{
    //walk
    Vector3 v = (pos - transform.GetPosition()).Normalised();
    v.y = 0.0f;
    physicsObject->AddForce(v * 40.0f);

    //rotate
    Vector3 ogRotEuler = Quaternion(Matrix4::BuildViewMatrix(transform.GetPosition(), pos, Vector3(0, 1, 0)).Inverse()).ToEuler();
    Quaternion finalRot = Quaternion::EulerAnglesToQuaternion(ogRotEuler.x, ogRotEuler.y, ogRotEuler.z);
    transform.SetOrientation(Quaternion::Slerp(transform.GetOrientation(), finalRot, 5.0f * dt));
}

void AIEnemy::FindKeys(float dt)
{
    float x, y, z;
    keysFound = game->GetKeysFound();
    for (auto i : keysFound) {
        if (i.second == 0) {
            istringstream iss(i.first);
            string token;
            if (getline(iss, token, '/')) { x = stof(token); }
            if (getline(iss, token, '/')) { y = stof(token); }
            if (getline(iss, token, '/')) { z = stof(token); }
            destinationPos = Vector3(x, y, z);
            break;
        }
        foundAllKeys = true;
    }
    Pathfinding(transform.GetPosition(), destinationPos);
    if (pathNodes.size() >= 2)
    {
        float nodeDistance = (pathNodes[1] - transform.GetPosition()).Length();
        if (nodeDistance <= 10)
        {
            Move(pathNodes[1], dt);
        }
    }
}

void AIEnemy::GotoFinalTreasure(float dt)
{
    Pathfinding(transform.GetPosition(), game->GetFinalTreasurePos());
    if (pathNodes.size() >= 2)
    {
        float nodeDistance = (pathNodes[1] - transform.GetPosition()).Length();
        if (nodeDistance <= 10)
        {
            Move(pathNodes[1], dt);
        }
    }
    else {
        gotoSuccess = true;
    }
}

void AIEnemy::Respawn()
{
    transform.SetPosition(originalPos);
    physicsObject->SetLinearVelocity(Vector3(0, 0, 0));
    rootSequence->Reset();
    foundAllKeys = false;
}

void AIEnemy::GenerateBehaviourTree() {
    //Treasure Sequence
    BehaviourAction* collectKey = new BehaviourAction("Collect Key",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {
                cout << "Start Collect Key" << endl;
                state = Ongoing;
            }
            else if (state == Ongoing) {
                if (foundAllKeys) {
                    return Success;
                }
                else {
                    FindKeys(dt);
                }
            }
            return state; // will be ¡¯ ongoing ¡¯ until success
        }
    );
    BehaviourAction* goToFinalTreasure = new BehaviourAction("Go To Final Treasure",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {
                cout << "Start Go To Final Treasure" << endl;
                state = Ongoing;
            }
            else if (state == Ongoing) {
                if (gotoSuccess) {
                    return Success;
                }
                else {
                    GotoFinalTreasure(dt);
                }
            }
            return state; // will be ¡¯ ongoing ¡¯ until success
        }
    );
    //Chase Sequence
    BehaviourAction* protectTreasure = new BehaviourAction("Protect Treasure",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {

                return Ongoing;
            }
            else if (state == Ongoing) {
                if (1) {

                    return Success;
                }
                return Failure;
            }
            return state;
        }
    );
    BehaviourAction* chasingPlayer = new BehaviourAction("Chasing Player",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {

                return Ongoing;
            }
            else if (state == Ongoing) {

                return Failure;
            }
            return state;
        }
    );

    sequence = new BehaviourSequence("Treasure Sequence");
    sequence->AddChild(collectKey);
    sequence->AddChild(goToFinalTreasure);

    selection = new BehaviourSelector("Chase Selection");
    selection->AddChild(protectTreasure);
    selection->AddChild(chasingPlayer);

    rootSequence = new BehaviourSequence("Root Sequence");
    rootSequence->AddChild(sequence);
    //rootSequence->AddChild(selection);

}

void AIEnemy::Update(float dt)
{
    //FindKeys(dt);
    rootSequence->Execute(dt);
    //BehaviourTree();
}

void AIEnemy::OnCollisionBegin(GameObject* otherObject)
{
    if (otherObject->GetName() == "treasure")
    {
        Vector3 p = otherObject->GetTransform().GetPosition();
        string s = to_string(p.x) + "/" + to_string(p.y) + "/" + to_string(p.z);
        if (game->GetKeysFound()[s] == 0) {
            game->GetKeysFound()[s] = 2;
            otherObject->GetRenderObject()->SetColour(Vector4(1, 0, 0, 1));
        }
        physicsObject->ApplyLinearImpulse(
            Vector3(rand() % 50 < 25 ? 10 : -10, 0, rand() % 50 < 25 ? 10 : -10));
    }
}