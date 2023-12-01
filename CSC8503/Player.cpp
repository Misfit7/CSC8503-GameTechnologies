#include "Player.h"
#include "..\CSC8503CoreClasses\PhysicsObject.h"
#include "..\CSC8503CoreClasses\RenderObject.h"
#include "..\NCLCoreClasses\Quaternion.h"

using namespace NCL;
using namespace CSC8503;

Player::Player(CourseWork& g, const Vector3& position,
    Mesh* mesh, Texture* basicTex, Shader* basicShader) :game(g), world(g.GetWorld())
{
    float meshSize = 1.0f;
    float inverseMass = 1.0f;

    SphereVolume* volume = new SphereVolume(1.0f);
    SetBoundingVolume((CollisionVolume*)volume);

    transform
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position);

    renderObject = new RenderObject(&transform, mesh, nullptr, basicShader);

    physicsObject = new PhysicsObject(&transform, GetBoundingVolume());
    physicsObject->SetInverseMass(inverseMass);
    physicsObject->InitSphereInertia();

    game.GetWorld()->AddGameObject(this);

}

void Player::Update(float dt)
{
    if (!game.GetSwitchCamera()) {
        Vector3 playerPos = transform.GetPosition();
        Vector3 linearImpulse;

        //check isStand
        Ray yRay = Ray(playerPos, Vector3(0, -1, 0));
        RayCollision floorCollision;
        if (world->Raycast(yRay, floorCollision, true, this))
        {
            float distance = (floorCollision.collidedAt - playerPos).Length();
            distance <= 1.01f ? isStand = 1 : isStand = 0;
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::W)) {
            linearImpulse.z = -1.0f;
            //player->GetPhysicsObject()->AddForce(Vector3(0.0f, 0.0f, -10.0f));
        }
        if (Window::GetKeyboard()->KeyDown(KeyCodes::S)) {
            linearImpulse.z = 1.0f;
        }
        if (Window::GetKeyboard()->KeyDown(KeyCodes::A)) {
            linearImpulse.x = -1.0f;
        }
        if (Window::GetKeyboard()->KeyDown(KeyCodes::D)) {
            linearImpulse.x = 1.0f;
        }
        //calculate move forward angle
        if (linearImpulse.Length() != 0.0f)
        {
            float newOrientation = RadiansToDegrees(atan2(-linearImpulse.x, -linearImpulse.z)) + world->GetMainCamera().GetYaw();
            Quaternion newOrientationQ = Quaternion::EulerAnglesToQuaternion(0, newOrientation, 0);

            transform.SetOrientation(Quaternion::Slerp(transform.GetOrientation(), newOrientationQ, 5 * dt));
            physicsObject->AddForce(newOrientationQ * Vector3(0, 0, -1.0f).Normalised() * 20);
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

        //interact
        if (Window::GetKeyboard()->KeyHeld(KeyCodes::E)) {

        }

        //move mode
        if (Window::GetKeyboard()->KeyHeld(KeyCodes::LSHIFT)) {

        }
        if (Window::GetKeyboard()->KeyHeld(KeyCodes::LMENU)) {

        }

        //scale
        if (Window::GetKeyboard()->KeyDown(KeyCodes::PLUS)) {

        }
        if (Window::GetKeyboard()->KeyDown(KeyCodes::MINUS)) {

        }

    }
}
