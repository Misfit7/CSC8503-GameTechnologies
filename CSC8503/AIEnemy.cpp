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

    InitPatrolPos();
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
    int count = 0;
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
        ++count;
    }
    if (count == keysFound.size()) {
        foundAllKeys = true;
        return;
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

void AIEnemy::InitPatrolPos() {
    for (auto i : game->GetSafePos())
        if ((i - transform.GetPosition()).Length() <= 25)
            patrolPos.emplace_back(i);
    patrolPos.emplace_back(game->GetFinalTreasurePos());
}

void AIEnemy::ProtectTreasure(float dt)
{
    if (!findPatrolPos) {
        patrolDes = patrolPos[rand() % patrolPos.size()];
        findPatrolPos = true;
        getToPatrolPos = false;
    }
    Pathfinding(transform.GetPosition(), patrolDes);
    if (pathNodes.size() >= 2)
    {
        float nodeDistance = (pathNodes[1] - transform.GetPosition()).Length();
        if (nodeDistance <= 10)
        {
            Move(pathNodes[1], dt);
        }
    }
    else {
        getToPatrolPos = true;
        findPatrolPos = false;
    }
}

void AIEnemy::ChasingPlayer(float dt)
{
    Vector3 enemyPos = transform.GetPosition();
    enemyPos.y = 1;
    Vector3 playerPos = game->GetPlayer()->GetTransform().GetPosition();
    playerPos.y = 1;
    playerPos.x += 2.5f;
    playerPos.z += 2.5f;
    Pathfinding(enemyPos, playerPos);
    if (pathNodes.size() >= 2)
    {
        float nodeDistance = (pathNodes[1] - transform.GetPosition()).Length();
        if (nodeDistance <= 10)
        {
            Move(pathNodes[1], dt);
        }
    }
    else if ((transform.GetPosition() - game->GetPlayer()->GetTransform().GetPosition()).Length() <= 10) {
        Move(game->GetPlayer()->GetTransform().GetPosition(), dt);
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
    //
    /*BehaviourAction* initSeletion = new BehaviourAction("Init Child",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Success) {
                cout << "Start Init" << endl;
                selection->Reset();
            }
            return state;
        }
    );*/
    //Chase Sequence
    BehaviourAction* protectTreasure = new BehaviourAction("Protect Treasure",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {
                cout << "Start Protect Treasure" << endl;
                return Ongoing;
            }
            else if (state == Ongoing) {
                if (getToPatrolPos) {
                    cout << "Patrol Success" << endl;
                    getToPatrolPos = false;
                    return Success;
                }
                else if ((game->GetPlayer()->GetTransform().GetPosition() - transform.GetPosition()).Length() > 15
                    && !game->GetPlayer()->GetKey()) {
                    ProtectTreasure(dt);
                    //cout << "AIEnemy Protecting Treasure" << endl;
                }
                else {
                    //cout << "Stop Protect Treasure" << endl;
                    return Failure;
                }
            }
            else if (state == Success || state == Failure) {
                state = Ongoing;
            }
            return state;
        }
    );
    BehaviourAction* chasingPlayer = new BehaviourAction("Chasing Player",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {
                cout << "Start Chasing Player" << endl;
                return Ongoing;
            }
            else if (state == Ongoing) {
                if (getToPlayer)
                {
                    //cout << "Chasing Player Success" << endl;
                    getToPlayer = false;
                    return Success;
                }
                else if ((game->GetPlayer()->GetTransform().GetPosition() - transform.GetPosition()).Length() <= 15
                    || game->GetPlayer()->GetKey()) {
                    ChasingPlayer(dt);
                }
                else {
                    //cout << "AIEnemy Stop Chasing Player" << endl;
                    return Failure;
                }
            }
            else if (state == Success) {
                state = Ongoing;
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

    /*sequence1 = new BehaviourSequence("Protect Sequence");
    sequence1->AddChild(selection);*/

    rootSequence = new BehaviourSequence("Root Sequence");
    rootSequence->AddChild(sequence);
    //rootSequence->AddChild(sequence1);
    rootSequence->AddChild(selection);

}

void AIEnemy::Update(float dt)
{
    if (game->GetPlayer()->GetHealth() == 0) getToPlayer = true;
    rootSequence->Execute(dt);
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
    if (otherObject == game->GetPlayer()) {
        game->GetPlayer()->SetHealth(0);
    }
}