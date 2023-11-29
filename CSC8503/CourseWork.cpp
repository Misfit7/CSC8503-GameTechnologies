#include "CourseWork.h"
#include "GameWorld.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "TextureLoader.h"

#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "StateGameObject.h"



using namespace NCL;
using namespace CSC8503;

CourseWork::CourseWork() : controller(*Window::GetWindow()->GetKeyboard(), *Window::GetWindow()->GetMouse()) {
    world = new GameWorld();
    window = Window::GetWindow();
#ifdef USEVULKAN
    renderer = new GameTechVulkanRenderer(*world);
    renderer->Init();
    renderer->InitStructures();
#else 
    renderer = new GameTechRenderer(*world);
#endif

    physics = new PhysicsSystem(*world);

    forceMagnitude = 10.0f;
    inSelectionMode = false;

    InitMainCamera();
    world->GetMainCamera().SetController(controller);

    controller.MapAxis(0, "Sidestep");
    controller.MapAxis(1, "UpDown");
    controller.MapAxis(2, "Forward");

    controller.MapAxis(3, "XLook");
    controller.MapAxis(4, "YLook");

    InitialiseAssets();
}

/*

Each of the little demo scenarios used in the game uses the same 2 meshes,
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void CourseWork::InitialiseAssets() {
    cubeMesh = renderer->LoadMesh("cube.msh");
    sphereMesh = renderer->LoadMesh("sphere.msh");
    charMesh = renderer->LoadMesh("goat.msh");
    enemyMesh = renderer->LoadMesh("Keeper.msh");
    bonusMesh = renderer->LoadMesh("Cylinder.msh");
    capsuleMesh = renderer->LoadMesh("capsule.msh");

    basicTex = renderer->LoadTexture("checkerboard.png");
    basicShader = renderer->LoadShader("scene.vert", "scene.frag");

    Menu();
}

CourseWork::~CourseWork() {
    delete cubeMesh;
    delete sphereMesh;
    delete charMesh;
    delete enemyMesh;
    delete bonusMesh;

    delete basicTex;
    delete basicShader;

    delete physics;
    delete renderer;
    delete world;
}

void CourseWork::UpdateGame(float dt) {
    UpdateKeys();

    world->UpdateWorld(dt);
    renderer->Update(dt);
    physics->Update(dt);

    if (currentGame == 1) {
        GameOneRunning(dt);
    }
    if (currentGame == 2) {
        GameTwoRunning(dt);
    }

    renderer->Render();
    Debug::UpdateRenderables(dt);

    // Display main menu
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::ESCAPE)) {
        Menu();
    }
}

void CourseWork::UpdateKeys() {
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::Y)) { //switch Camera
        switchCamera = !switchCamera;
        if (switchCamera) { InitMainCamera(); }
        else if (!switchCamera) { InitPlayerCamera(); }
    }

    //Running certain physics updates in a consistent order might cause some
    //bias in the calculations - the same objects might keep 'winning' the constraint
    //allowing the other one to stretch too much etc. Shuffling the order so that it
    //is random every frame can help reduce such bias.
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::F9)) {
        world->ShuffleConstraints(true);
    }
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::F10)) {
        world->ShuffleConstraints(false);
    }

    if (Window::GetKeyboard()->KeyPressed(KeyCodes::F7)) {
        world->ShuffleObjects(true);
    }
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::F8)) {
        world->ShuffleObjects(false);
    }

    DebugObjectMovement();
}

void CourseWork::DebugObjectMovement() {
    //If we've selected an object, we can manipulate it with some key presses
    if (inSelectionMode && selectionObject) {
        //Twist the selected object!
        if (Window::GetKeyboard()->KeyDown(KeyCodes::LEFT)) {
            selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::RIGHT)) {
            selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM7)) {
            selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM8)) {
            selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::RIGHT)) {
            selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::UP)) {
            selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::DOWN)) {
            selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
        }

        if (Window::GetKeyboard()->KeyDown(KeyCodes::NUM5)) {
            selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
        }
    }
}

void CourseWork::InitMainCamera() {
    mainCamera = new PerspectiveCamera();
    world->SetMainCamera(mainCamera);
    world->GetMainCamera().SetController(controller);

    world->GetMainCamera().SetNearPlane(0.1f);
    world->GetMainCamera().SetFarPlane(500.0f);
    world->GetMainCamera().SetPitch(-15.0f);
    world->GetMainCamera().SetYaw(315.0f);
    world->GetMainCamera().SetPosition(Vector3(-60, 40, 60));

}

void CourseWork::InitPlayerCamera() {
    playerCamera = new PlayerCamera(*world, *player);
    world->SetMainCamera(playerCamera);
    world->GetMainCamera().SetController(controller);

    world->GetMainCamera().SetNearPlane(0.1f);
    world->GetMainCamera().SetFarPlane(1500.0f);
    world->GetMainCamera().SetPitch(0.0f);
    world->GetMainCamera().SetYaw(0.0f);

}

void CourseWork::InitGameOne() {
    currentGame = 1;
    world->ClearAndErase();
    physics->Clear();
    physics->UseGravity(useGravity);

    InitDefaultFloor();
    InitGameOneObject();

    InitPlayerCamera();

}

/*

A single function to add a large immoveable cube to the bottom of our world

*/
GameObject* CourseWork::AddFloorToWorld(const Vector3& position) {
    GameObject* floor = new GameObject();

    Vector3 floorSize = Vector3(200, 2, 200);
    AABBVolume* volume = new AABBVolume(floorSize);
    floor->SetBoundingVolume((CollisionVolume*)volume);
    floor->GetTransform()
        .SetScale(floorSize * 2)
        .SetPosition(position);

    floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));
    floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

    floor->GetPhysicsObject()->SetInverseMass(0);
    floor->GetPhysicsObject()->InitCubeInertia();

    world->AddGameObject(floor);

    return floor;
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple'
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* CourseWork::AddSphereToWorld(const Vector3& position, float radius, float inverseMass) {
    GameObject* sphere = new GameObject();

    Vector3 sphereSize = Vector3(radius, radius, radius);
    SphereVolume* volume = new SphereVolume(radius);
    sphere->SetBoundingVolume((CollisionVolume*)volume);

    sphere->GetTransform()
        .SetScale(sphereSize)
        .SetPosition(position);

    sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
    sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

    sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
    sphere->GetPhysicsObject()->InitSphereInertia();

    world->AddGameObject(sphere);

    return sphere;
}

GameObject* CourseWork::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
    GameObject* cube = new GameObject();

    AABBVolume* volume = new AABBVolume(dimensions);
    cube->SetBoundingVolume((CollisionVolume*)volume);

    cube->GetTransform()
        .SetPosition(position)
        .SetScale(dimensions * 2);

    cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
    cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

    cube->GetPhysicsObject()->SetInverseMass(inverseMass);
    cube->GetPhysicsObject()->InitCubeInertia();

    world->AddGameObject(cube);

    return cube;
}

GameObject* CourseWork::AddBoardToWorld(const Vector3& position, const Vector3& rotation, const Vector3& boardSize, float inverseMass)
{
    GameObject* Board = new GameObject();
    AABBVolume* volume = new AABBVolume(boardSize);
    Board->SetBoundingVolume((CollisionVolume*)volume);
    Board->GetTransform()
        .SetScale(boardSize * 2)
        .SetPosition(position)
        .SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(1.0f, 0.0f, 0.0f), rotation.x) *
            Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), rotation.y) *
            Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), rotation.z));

    Board->SetRenderObject(new RenderObject(&Board->GetTransform(), cubeMesh, basicTex, basicShader));
    Board->GetRenderObject()->SetColour(Vector4(0.0, 0, 0.7, 1));
    Board->SetPhysicsObject(new PhysicsObject(&Board->GetTransform(), Board->GetBoundingVolume()));

    Board->GetPhysicsObject()->SetInverseMass(inverseMass);
    Board->GetPhysicsObject()->InitCubeInertia();

    Board->SetLockFlags(AxisLock::ANGULAR_X | AxisLock::ANGULAR_Z | AxisLock::LINEAR_X | AxisLock::LINEAR_Y | AxisLock::LINEAR_Z);
    Board->SetUsesGravity(false);

    world->AddGameObject(Board);
    return Board;
}

GameObject* CourseWork::AddConstraintSphereToWorld(const Vector3& position, float radius, float inverseMass, int linkNum, int impulseNum) {
    if (linkNum < 2) return nullptr;
    if (impulseNum < 2 || impulseNum>linkNum) impulseNum = 2;
    float sphereSize = radius;

    float maxDistance = 3 * radius; // constraint distance
    float sphereDistance = 2 * radius; // distance between links

    GameObject* linkStart = AddSphereToWorld(position + Vector3(0.0f, 0.0f, 0.0f), sphereSize, 0.0f);
    GameObject* previous = linkStart;

    GameObject* impulseObject = new GameObject;

    for (int i = 2; i <= linkNum; ++i) {
        GameObject* sphere = AddSphereToWorld(position + Vector3(0.0f, i * -sphereDistance, 0.0f), sphereSize, inverseMass);
        PositionConstraint* constraint = new PositionConstraint(previous, sphere, maxDistance);
        world->AddConstraint(constraint);
        previous = sphere;
        if (i == impulseNum) impulseObject = sphere;
    }
    LinkMaxDistance = (linkNum - 1) * maxDistance;
    return impulseObject;
}

GameObject* CourseWork::AddPlayerToWorld(const Vector3& position) {
    float meshSize = 1.0f;
    float inverseMass = 0.4f;

    GameObject* character = new GameObject();
    SphereVolume* volume = new SphereVolume(1.0f);

    character->SetBoundingVolume((CollisionVolume*)volume);

    character->GetTransform()
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position);

    character->SetRenderObject(new RenderObject(&character->GetTransform(), charMesh, nullptr, basicShader));
    character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

    character->GetPhysicsObject()->SetInverseMass(inverseMass);
    character->GetPhysicsObject()->InitSphereInertia();

    world->AddGameObject(character);

    return character;
}

GameObject* CourseWork::AddEnemyToWorld(const Vector3& position) {
    float meshSize = 3.0f;
    float inverseMass = 0.5f;

    GameObject* character = new GameObject();

    AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);
    character->SetBoundingVolume((CollisionVolume*)volume);

    character->GetTransform()
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position);

    character->SetRenderObject(new RenderObject(&character->GetTransform(), enemyMesh, nullptr, basicShader));
    character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

    character->GetPhysicsObject()->SetInverseMass(inverseMass);
    character->GetPhysicsObject()->InitSphereInertia();

    world->AddGameObject(character);

    return character;
}

GameObject* CourseWork::AddBonusToWorld(const Vector3& position) {
    float meshSize = 1.0f;
    float inverseMass = 0.5f;

    GameObject* bonus = new GameObject();

    SphereVolume* volume = new SphereVolume(0.5f * meshSize);
    bonus->SetBoundingVolume((CollisionVolume*)volume);

    bonus->GetTransform()
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position);

    bonus->SetRenderObject(new RenderObject(&bonus->GetTransform(), bonusMesh, basicTex, basicShader));
    bonus->SetPhysicsObject(new PhysicsObject(&bonus->GetTransform(), bonus->GetBoundingVolume()));

    bonus->GetPhysicsObject()->SetInverseMass(inverseMass);
    bonus->GetPhysicsObject()->InitSphereInertia();

    world->AddGameObject(bonus);

    return bonus;
}

void CourseWork::InitDefaultFloor() {
    AddFloorToWorld(Vector3(0, 0, 0));
}

void CourseWork::InitGameOneObject() {
    player = AddPlayerToWorld(Vector3(-10, 5, 0));
    AddEnemyToWorld(Vector3(-5, 5, 0));
    AddBonusToWorld(Vector3(-15, 5, 0));
    LinkImpulseObject = AddConstraintSphereToWorld(Vector3(-20, 30, 0), 1, 2, 6, 6);
    AddBoardToWorld(Vector3(-25, 7, 0), Vector3(0, 0, 0), Vector3(5, 5, 1));
}

/*
Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around.

*/
bool CourseWork::SelectObject() {
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::E)) {
        inSelectionMode = !inSelectionMode;
        if (inSelectionMode) {
            Window::GetWindow()->ShowOSPointer(true);
            Window::GetWindow()->LockMouseToWindow(false);
        }
        else {
            Window::GetWindow()->ShowOSPointer(false);
            Window::GetWindow()->LockMouseToWindow(true);
        }
    }
    if (inSelectionMode) {
        Debug::Print("Press E to change to camera mode!", Vector2(5, 85));

        if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::Left)) {
            if (selectionObject) {	//set colour to deselected;
                selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
                cout << selectionObject->GetTransform().GetPosition();
                selectionObject = nullptr;
            }

            Ray ray = CollisionDetection::BuildRayFromMouse(world->GetMainCamera());

            RayCollision closestCollision;
            if (world->Raycast(ray, closestCollision, true)) {
                selectionObject = (GameObject*)closestCollision.node;

                selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));

                Debug::DrawLine(ray.GetPosition(), closestCollision.collidedAt, Debug::WHITE, 5.0f);
                Debug::DrawLine(closestCollision.collidedAt, closestCollision.collidedAt + closestCollision.collidedNormal, Debug::YELLOW, 5.0f);

                return true;
            }
            else {
                return false;
            }
        }
    }
    else {
        Debug::Print("Press E to change to select mode!", Vector2(5, 85));
    }
    return false;
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/

void CourseWork::MoveSelectedObject() {
    Debug::Print("Click Force:" + std::to_string(forceMagnitude), Vector2(5, 90));
    forceMagnitude += Window::GetMouse()->GetWheelMovement() * 100.0f;

    if (!selectionObject) {
        return;//we haven't selected anything!
    }
    //Push the selected object!
    if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::Right)) {
        Ray ray = CollisionDetection::BuildRayFromMouse(world->GetMainCamera());

        RayCollision closestCollision;
        if (world->Raycast(ray, closestCollision, true)) {
            if (closestCollision.node == selectionObject) {
                selectionObject->GetPhysicsObject()->AddForceAtPosition(ray.GetDirection() * forceMagnitude, closestCollision.collidedAt);
            }
        }
    }
}

void CourseWork::Menu(const std::string& text, const Vector4& colour) {
    PushdownMachine machine(new MainMenu(this));

    while (window->UpdateWindow()) {
        float dt = window->GetTimer().GetTimeDeltaSeconds();
        Debug::Print(text, Vector2(50.0f - text.length(), 20.0f), colour);
        if (!machine.Update(dt)) {
            return;
        }
        renderer->Update(dt);
        renderer->Render();
        Debug::UpdateRenderables(dt);
    }
}

void CourseWork::UpdateLinkConstraintObject() {
    int x;
    if (LinkImpulseObject->GetTransform().GetPosition().y < 30 - LinkMaxDistance + 2) {
        rand() % 2 ? x = 1 : x = -1;
        LinkImpulseObject->GetPhysicsObject()->ApplyLinearImpulse(Vector3(0.0f, 0.0f, 20.0f * x));
    }
}

void CourseWork::ShowUI() {

    return;
}

void CourseWork::PlayerUpdate(float dt) {
    if (!switchCamera) {
        Vector3 playerPos = player->GetTransform().GetPosition();
        Vector3 linearImpulse;

        //check isStand
        Ray yRay = Ray(playerPos, Vector3(0, -1, 0));
        RayCollision floorCollision;
        if (world->Raycast(yRay, floorCollision, true, player))
        {
            float distance = (floorCollision.collidedAt - playerPos).Length();
            distance <= 1.0f ? isStand = 1 : isStand = 0;
        }
        //clear elasticity
        Vector3 playerLV = player->GetPhysicsObject()->GetLinearVelocity();
        if (isStand) player->GetPhysicsObject()->SetLinearVelocity(Vector3(playerLV.x, 0, playerLV.z));

        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::W)) {
            linearImpulse.z = -1.0f;
            //player->GetPhysicsObject()->AddForce(Vector3(0.0f, 0.0f, -10.0f));
        }
        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::S)) {
            linearImpulse.z = 1.0f;
        }
        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::A)) {
            linearImpulse.x = -1.0f;
        }
        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::D)) {
            linearImpulse.x = 1.0f;
        }
        //calculate move forward angle
        if (linearImpulse.Length() != 0.0f)
        {
            float newOrientation = RadiansToDegrees(atan2(-linearImpulse.x, -linearImpulse.z)) + world->GetMainCamera().GetYaw();
            Quaternion newOrientationQ = Quaternion::EulerAnglesToQuaternion(0, newOrientation, 0);

            player->GetTransform().SetOrientation(Quaternion::Slerp(player->GetTransform().GetOrientation(), newOrientationQ, 5 * dt));
            player->GetPhysicsObject()->AddForce(newOrientationQ * Vector3(0, 0, -1.0f).Normalised() * 20);
        }

        if (Window::GetKeyboard()->KeyHeld(NCL::KeyCodes::LSHIFT)) {

        }
        if (Window::GetKeyboard()->KeyHeld(NCL::KeyCodes::LMENU)) {

        }

        // double jump
        if (jumpCount >= 1 && isStand)
            jumpCount = 0;
        if (Window::GetKeyboard()->KeyPressed(NCL::KeyCodes::SPACE)) {
            if (isStand) {
                player->GetPhysicsObject()->ApplyLinearImpulse(Vector3(0, 1, 0) * 40);
                ++jumpCount;
            }
            else if (jumpCount < 2) {
                player->GetPhysicsObject()->ApplyLinearImpulse(Vector3(0, 1, 0) * 30);
                ++jumpCount;
            }
        }
        cout << jumpCount << endl;

        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::PLUS)) {

        }
        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::MINUS)) {

        }


    }
}

void CourseWork::GameOneRunning(float dt)
{
    ShowUI();

    PlayerUpdate(dt);

    UpdateLinkConstraintObject();

    if (!inSelectionMode) {
        world->GetMainCamera().UpdateCamera(dt);
    }

    RayCollision closestCollision;
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::K) && selectionObject) {
        Vector3 rayPos;
        Vector3 rayDir;

        rayDir = selectionObject->GetTransform().GetOrientation() * Vector3(0, 0, -1);

        rayPos = selectionObject->GetTransform().GetPosition();

        Ray r = Ray(rayPos, rayDir);

        if (world->Raycast(r, closestCollision, true, selectionObject)) {
            if (objClosest) {
                objClosest->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
            }
            objClosest = (GameObject*)closestCollision.node;

            objClosest->GetRenderObject()->SetColour(Vector4(1, 0, 1, 1));
        }
    }

    Debug::DrawLine(Vector3(), Vector3(0, 100, 0), Vector4(1, 0, 0, 1));

    SelectObject();
    MoveSelectedObject();
}

void CourseWork::GameTwoRunning(float dt)
{

}

void CourseWork::SetGameState(int value) {
    if (value == 1) {
        InitGameOne();
    }
    else if (value == 2) {
        InitGameOne();
    }
    else if (value == 3) {
        Window::DestroyGameWindow();
        exit(0);
    }
}
