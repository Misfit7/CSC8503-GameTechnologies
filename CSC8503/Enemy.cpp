#include "Enemy.h"

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

Enemy::Enemy(CourseWork& g, const Vector3& position, Mesh* mesh, Texture* basicTex, Shader* basicShader,
    float inverseMass, string t)
    :game(g), world(g.GetWorld()), grid(g.GetGrid())
{
    type = t;
    originalPos = position;
    grid->PrintAllNodes();

    float meshSize = 3.0f;

    AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);
    SetBoundingVolume((CollisionVolume*)volume);

    transform
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position);

    if (type == "v") {
        if (transform.GetPosition().y >= 50.0f) {
            transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 180.0f) *
                Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), 180.0f));
        }
        else if (transform.GetPosition().y <= 50.0f) {
            transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 180.0f) *
                Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), 0.0f));
        }
        moveDirection = "forward";
    }
    else if (type == "h")
    {
        if (transform.GetPosition().y >= 50.0f) {
            transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 90.0f) *
                Quaternion::AxisAngleToQuaterion(Vector3(1.0f, 0.0f, 0.0f), 180.0f));
        }
        else if (transform.GetPosition().y <= 50.0f) {
            transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 90.0f) *
                Quaternion::AxisAngleToQuaterion(Vector3(1.0f, 0.0f, 0.0f), 0.0f));
        }
        moveDirection = "left";
    }

    renderObject = new RenderObject(&transform, mesh, nullptr, basicShader);
    renderObject->SetColour(Vector4(1, 0, 0, 1));

    physicsObject = new PhysicsObject(&transform, GetBoundingVolume());
    physicsObject->SetInverseMass(inverseMass);
    physicsObject->InitCubeInertia();
    SetUsesGravity(false);

    GenerateStateMachine();

    world->AddGameObject(this);
}

void Enemy::GenerateStateMachine()
{
    stateMachine = new StateMachine();
    State* wander = new State([&](float dt)-> void
        {
            //cout << "now is wandering" << endl;
            this->Wander(dt);
        }
    );
    State* chase = new State([&](float dt)-> void
        {
            //cout << "now is chasing" << endl;
            this->ChasePlayer(dt);;
        }
    );
    State* dead = new State([&](float dt)-> void
        {
            this->Respawn();
        }
    );
    stateMachine->AddState(wander);
    stateMachine->AddState(chase);
    stateMachine->AddState(dead);

    stateMachine->AddTransition(new StateTransition(wander, chase,
        [&]()-> bool
        {
            return ((game.GetPlayer()->GetTransform().GetPosition() - (transform.GetPosition())
                ).Length() <= 15.0f);
        }
    ));
    stateMachine->AddTransition(new StateTransition(chase, dead,
        [&]()-> bool
        {
            if (game.GetPlayer()->GetHealth() == 0 || ((game.GetPlayer()->GetTransform().GetPosition() - (transform.GetPosition())
                ).Length() > 20.0f))
                return  true;
        }
    ));
    stateMachine->AddTransition(new StateTransition(dead, wander,
        [&]()-> bool
        {
            return 1;
        }
    ));
}

void Enemy::Wander(float dt)
{
    if (type == "v") {
        if (aliveTime < 0.0f)
        {
            moveDirection = "forward";
            if (transform.GetPosition().y >= 50.0f) {
                transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 180.0f) *
                    Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), 180.0f));
            }
            else if (transform.GetPosition().y <= 50.0f) {
                transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 180.0f) *
                    Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), 0.0f));
            }
        }
        else if (aliveTime > 2.0f)
        {
            moveDirection = "back";
            if (transform.GetPosition().y >= 50.0f) {
                transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 0.0f) *
                    Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), 180.0f));
            }
            else if (transform.GetPosition().y <= 50.0f) {
                transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 0.0f) *
                    Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), 0.0f));
            }
        }
        if (moveDirection == "forward") {
            //cout << "im walking forward" << endl;
            aliveTime += dt;
            Vector3 newPos = transform.GetPosition();
            newPos.z += 0.1;
            transform.SetPosition(newPos);
        }
        else if (moveDirection == "back") {
            //cout << "im walking back" << endl;
            aliveTime -= dt;
            Vector3 newPos = transform.GetPosition();
            newPos.z -= 0.1;
            transform.SetPosition(newPos);
        }
    }
    else if (type == "h") {
        if (aliveTime < 0.0f)
        {
            moveDirection = "left";
            if (transform.GetPosition().y >= 50.0f) {
                transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 90.0f) *
                    Quaternion::AxisAngleToQuaterion(Vector3(1.0f, 0.0f, 0.0f), 180.0f));
            }
            else if (transform.GetPosition().y <= 50.0f) {
                transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 90.0f) *
                    Quaternion::AxisAngleToQuaterion(Vector3(1.0f, 0.0f, 0.0f), 0.0f));
            }
        }
        else if (aliveTime > 2.0f)
        {
            moveDirection = "right";
            if (transform.GetPosition().y >= 50.0f) {
                transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 270.0f) *
                    Quaternion::AxisAngleToQuaterion(Vector3(1.0f, 0.0f, 0.0f), 180.0f));
            }
            else if (transform.GetPosition().y <= 50.0f) {
                transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 270.0f) *
                    Quaternion::AxisAngleToQuaterion(Vector3(1.0f, 0.0f, 0.0f), 0.0f));
            }
        }
        if (moveDirection == "left") {
            //cout << "im walking forward" << endl;
            aliveTime += dt;
            Vector3 newPos = transform.GetPosition();
            newPos.x += 0.1;
            transform.SetPosition(newPos);
        }
        else if (moveDirection == "right") {
            //cout << "im walking back" << endl;
            aliveTime -= dt;
            Vector3 newPos = transform.GetPosition();
            newPos.x -= 0.1;
            transform.SetPosition(newPos);
        }
    }
    ogQ = transform.GetOrientation();
    //cout << aliveTime << endl;
}

void Enemy::ChasePlayer(float dt)
{
    Move(game.GetPlayer()->GetTransform().GetPosition(), dt);
}

void Enemy::Move(const Vector3& pos, float dt)
{
    //walk
    Vector3 v = (pos - transform.GetPosition()).Normalised();
    v.y = 0.0f;
    Vector3 ogRotEuler;
    if (transform.GetPosition().y >= 50.0f)
    {
        physicsObject->AddForce(v * -40.0f);
        ogRotEuler = Quaternion(Matrix4::BuildViewMatrix(transform.GetPosition(), pos, Vector3(0, -1, 0)).Inverse()).ToEuler();

    }
    else if (transform.GetPosition().y <= 50.0f) {
        physicsObject->AddForce(v * 40.0f);
        ogRotEuler = Quaternion(Matrix4::BuildViewMatrix(transform.GetPosition(), pos, Vector3(0, 1, 0)).Inverse()).ToEuler();

    }
    //rotate
    Quaternion finalRot = Quaternion::EulerAnglesToQuaternion(ogRotEuler.x, ogRotEuler.y, ogRotEuler.z);
    transform.SetOrientation(Quaternion::Slerp(ogQ, finalRot, 5.0f * dt));
    ogQ = transform.GetOrientation();
}

void Enemy::Respawn()
{
    transform.SetPosition(originalPos);
    aliveTime = 0;
}

void Enemy::Update(float dt)
{
    stateMachine->Update(dt);
}

void Enemy::OnCollisionBegin(GameObject* otherObject)
{
    if (otherObject == game.GetPlayer())
        game.GetPlayer()->SetHealth(0);
}
