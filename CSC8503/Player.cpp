#include "Player.h"
#include "NetworkObject.h"
#include "..\CSC8503CoreClasses\PhysicsObject.h"
#include "..\CSC8503CoreClasses\RenderObject.h"
#include "..\NCLCoreClasses\Quaternion.h"

using namespace NCL;
using namespace CSC8503;

int Player::playerNum = 0;

Player::Player(CourseWork& g, const Vector3& position,
    Mesh* mesh, Texture* basicTex, Shader* basicShader) :game(g), world(g.GetWorld())
{
    playerNum++;
    networkObject = new NetworkObject(*this, playerNum);

    float meshSize = 1.0f;
    float inverseMass = 1.0f;
    iMass = inverseMass;

    SphereVolume* volume = new SphereVolume(1.0f);
    SetBoundingVolume((CollisionVolume*)volume);

    transform
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position);
    renderObject = new RenderObject(&transform, mesh, nullptr, basicShader);
    renderObject->SetColour(Vector4(0.7, 0.7, 0.7, 1));

    physicsObject = new PhysicsObject(&transform, GetBoundingVolume());
    physicsObject->SetInverseMass(inverseMass);
    physicsObject->InitSphereInertia();

    world->AddGameObject(this);
}

void Player::Update(float dt)
{
    if (isHit) invincible += dt;
    if (invincible >= 1.0f) {
        invincible = 0.0f;
        isHit = false;
    }

    Vector3 playerPos = transform.GetPosition();
    Ray yRay = Ray(playerPos, Vector3(0, -1, 0));
    speed = 1.0f;
    //move mode
    if (Window::GetKeyboard()->KeyHeld(KeyCodes::SHIFT)) {
        speed = 2.0f;
    }

    if (transform.GetPosition().y >= 50.0f && !isStand) {
        //game.GetPlayerCamera()->SetDU(60.0f, -89.0f);
        game.GetPlayerCamera()->SetViewMat(-1, -1);
        yRay = Ray(playerPos, Vector3(0, 1, 0));
        physicsObject->SetInverseMass(-iMass);

        if (!switchOrientation) {
            switchOrientation = true;
            transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), 180.0f));
        }
    }
    else if (transform.GetPosition().y <= 50.0f && !isStand) {
        //game.GetPlayerCamera()->SetDU(-60.0f, 89.0f);
        game.GetPlayerCamera()->SetViewMat(1, 1);
        yRay = Ray(playerPos, Vector3(0, -1, 0));
        physicsObject->SetInverseMass(iMass);

        if (switchOrientation) {
            switchOrientation = false;
            transform.SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), 0.0f));
        }
    }

    RayCollision floorCollision;
    if (!game.GetSwitchCamera()) {
        Vector3 linearImpulse;

        //check isStand
        if (world->Raycast(yRay, floorCollision, true, this))
        {
            float distance = (floorCollision.collidedAt - playerPos).Length();
            distance <= 1.01f ? isStand = 1 : isStand = 0;
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::W)) {
            if (!switchOrientation) {
                linearImpulse.z = -speed;
            }
            else
                linearImpulse.z = speed;
            //player->GetPhysicsObject()->AddForce(Vector3(0.0f, 0.0f, -10.0f));
        }
        if (Window::GetKeyboard()->KeyDown(KeyCodes::S)) {
            if (!switchOrientation) {
                linearImpulse.z = speed;
            }
            else
                linearImpulse.z = -speed;
        }
        if (Window::GetKeyboard()->KeyDown(KeyCodes::A)) {
            linearImpulse.x = -speed;
        }
        if (Window::GetKeyboard()->KeyDown(KeyCodes::D)) {
            linearImpulse.x = speed;
        }
        //cout << linearImpulse;
        //calculate move forward angle
        if (linearImpulse.Length() != 0.0f)
        {
            float newOrientation = RadiansToDegrees(atan2(-linearImpulse.x, -linearImpulse.z)) + world->GetMainCamera().GetYaw();
            Quaternion newOrientationQ;
            if (!switchOrientation) {
                newOrientationQ = Quaternion::EulerAnglesToQuaternion(0, newOrientation, 0)
                    * Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), 0.0f);
                transform.SetOrientation(Quaternion::Slerp(transform.GetOrientation(), newOrientationQ, 5 * dt));
                physicsObject->AddForce(newOrientationQ * Vector3(0, 0, -1.0f).Normalised() * 20 * speed);

            }
            else if (switchOrientation) {
                newOrientationQ = Quaternion::EulerAnglesToQuaternion(0, newOrientation, 0)
                    * Quaternion::AxisAngleToQuaterion(Vector3(0.0f, -1.0f, 0.0f), 0.0f)
                    * Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, -1.0f), 180.0f);
                transform.SetOrientation(Quaternion::Slerp(transform.GetOrientation(), newOrientationQ, 5 * dt));
                physicsObject->AddForce(newOrientationQ * Vector3(0, 0, 1.0f).Normalised() * 20 * speed);

            }

        }

        // double jump
        if (jumpCount >= 1 && isStand)
        {
            //clear elasticity
            Vector3 playerLV = physicsObject->GetLinearVelocity();
            physicsObject->SetLinearVelocity(Vector3(playerLV.x, 0, playerLV.z));
            jumpCount = 0;
        }
        if (Window::GetKeyboard()->KeyPressed(KeyCodes::SPACE)) {
            if (isStand) {
                physicsObject->ApplyLinearImpulse(Vector3(0, 1, 0) * 20);
                ++jumpCount;
            }
            else if (jumpCount < 2) {
                physicsObject->ApplyLinearImpulse(Vector3(0, 1, 0) * 15);
                ++jumpCount;
            }
        }
        //cout << jumpCount << endl;

        Debug::Print("O", Vector2(49.5f, 49.5f), Debug::RED);
        //interact
        RayCollision grabCollision;
        if (Window::GetKeyboard()->KeyPressed(KeyCodes::Q)) {
            isGrab = !isGrab;
            if (isGrab)
            {
                Ray camRay = Ray(game.GetPlayerCamera()->GetPosition(), game.GetPlayerCamera()->GetFoward());

                if (world->Raycast(camRay, grabCollision, true, this))
                {
                    Vector3 grabDistance = transform.GetPosition() - grabCollision.collidedAt;
                    Vector3 force;
                    float forceDegree = 0.5;
                    if (!switchOrientation)
                    {
                        force = grabDistance * -forceDegree;
                    }
                    else if (switchOrientation)
                    {
                        force = grabDistance * forceDegree;
                    }

                    physicsObject->ApplyLinearImpulse(force);
                    Debug::DrawLine(transform.GetPosition(), grabCollision.collidedAt, Debug::BLACK, 0.5f);
                }
                isGrab = !isGrab;
            }
        }
        if (Window::GetKeyboard()->KeyDown(KeyCodes::F)) {

        }


        //scale
        if (Window::GetKeyboard()->KeyDown(KeyCodes::PLUS)) {

        }
        if (Window::GetKeyboard()->KeyDown(KeyCodes::MINUS)) {

        }

    }

    if (!switchOrientation)
        Pathfinding();

    if (health <= 0 || playerPos.y > 120.0f || playerPos.y < -10.0f) Respawn();
}

void Player::ResetKey() {
    world->RemoveConstraint(constraint);
    game.GetKey()->GetTransform().
        SetPosition(Vector3(12 * game.GetGrid()->GetNodeSize(), 96, 16 * game.GetGrid()->GetNodeSize()));
    game.GetKey()->GetPhysicsObject()->SetLinearVelocity(Vector3(0, 0, 0));
    getKey = false;
}

void Player::ResetFinalTreasure() {
    world->RemoveConstraint(constraint);
    game.GetFinalTreasure()->GetTransform().
        SetPosition(game.GetFinalTreasurePos());
    game.GetFinalTreasure()->GetPhysicsObject()->SetLinearVelocity(Vector3(0, 0, 0));
    getKey = false;
}

void Player::Respawn()
{
    health = 2;
    if (transform.GetPosition().y < 50.0f)
        transform.SetPosition(game.GetSafePos()[rand() % game.GetSafePos().size()]);
    else if (transform.GetPosition().y > 50.0f)
        transform.SetPosition(Vector3(game.GetGrid()->GetNodeSize(), 98, game.GetGrid()->GetNodeSize()));
    physicsObject->SetLinearVelocity(Vector3(0, 0, 0));
    if (getKey) {
        ResetKey();
        ResetFinalTreasure();
    }
}

void Player::Pathfinding() {
    Vector3 startPos = transform.GetPosition();
    startPos.x += 2.5f;
    startPos.z += 2.5f;
    Vector3 endPos = game.GetFinalTreasurePos();

    outPath.Clear();
    pathNodes.clear();
    bool found = game.GetGrid()->FindPath(startPos, endPos, outPath);

    Vector3 pos;
    while (outPath.PopWaypoint(pos)) {
        pathNodes.push_back(pos);
    }

    DisplayPathfinding();
}

void Player::DisplayPathfinding() {
    for (int i = 1; i < pathNodes.size(); ++i) {
        Vector3 a = pathNodes[i - 1];
        Vector3 b = pathNodes[i];

        Debug::DrawLine(a, b, Vector4(0, 1, 0, 1));
    }
}

void Player::OnCollisionBegin(GameObject* otherObject)
{
    if (otherObject->GetName() == "treasure")
    {
        Vector3 p = otherObject->GetTransform().GetPosition();
        string s = to_string(p.x) + "/" + to_string(p.y) + "/" + to_string(p.z);
        if (game.GetKeysFound()[s] == 0) {
            game.GetKeysFound()[s] = 1;
            otherObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
        }
    }
    else if (otherObject->GetName() == "key" && !getKey) {
        constraint = new PositionConstraint(otherObject, this, 2.0f);
        world->AddConstraint(constraint);
        getKey = true;
    }
}