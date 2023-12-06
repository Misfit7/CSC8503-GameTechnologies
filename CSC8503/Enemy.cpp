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

Enemy::Enemy(CourseWork& g, const Vector3& position, Mesh* mesh, Texture* basicTex, Shader* basicShader)
    :game(g), world(g.GetWorld()), grid(g.GetGrid())
{
    grid->PrintAllNodes();

    float meshSize = 3.0f;
    float inverseMass = 0.5f;

    AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);
    SetBoundingVolume((CollisionVolume*)volume);

    transform
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position)
        .SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 180.0f));

    renderObject = new RenderObject(&transform, mesh, nullptr, basicShader);
    renderObject->SetColour(Vector4(1, 0, 0, 1));

    physicsObject = new PhysicsObject(&transform, GetBoundingVolume());
    physicsObject->SetInverseMass(inverseMass);
    physicsObject->InitSphereInertia();

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
                ).Length() <= 20.0f);
        }
    ));
    stateMachine->AddTransition(new StateTransition(chase, wander,
        [&]()-> bool
        {
            return ((game.GetPlayer()->GetTransform().GetPosition() - (transform.GetPosition())
                ).Length() >= 20.0f);
        }
    ));
}

void Enemy::Wander(float dt)
{
    if (aliveTime < 0.0f)
    {
        moveDirection = "forward";
        transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 180.0f));
    }
    else if (aliveTime > 3.0f)
    {
        moveDirection = "back";
        transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), 0.0f));
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
    //cout << aliveTime << endl;
}

void Enemy::ChasePlayer(float dt)
{

}

void Enemy::Respawn()
{
}

void Enemy::Update(float dt)
{
    stateMachine->Update(dt);
}
