#include "CourseWork.h"
#include "GameWorld.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "TextureLoader.h"

#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "StateGameObject.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <ctime>

using namespace NCL;
using namespace CSC8503;

using namespace std;

const int blocksize = 5;
const int n = 20; //row
const int m = 20; //col

const int RATE_KEEP_DIR = 000;
const int TWIST_RATE = 10;
const int RETURN_RATE = 100;

int  Sx, Sy, Tx, Ty, Max;
char Map[n][m];
typedef pair<int, int> PII;
const int dx[4] = { -1,0,1,0 }, dy[4] = { 0,1,0,-1 };
const int Dx[4] = { -2,0,2,0 }, Dy[4] = { 0,2,0,-2 };

mt19937 RND(time(0));

//Check if (x,y) has a neighbour can go.
bool Check(const int x, const int y)
{
    for (int i = 0; i < 4; ++i)
    {
        int nx = x + dx[i], ny = y + dy[i], Nx = x + Dx[i], Ny = y + Dy[i];
        if (Nx >= 0 && Nx < n && Ny >= 0 && Ny < m && (Map[Nx][Ny] == 'x' && Map[nx][ny] == 'x'))return true;
    } return false;
}

void Dfs(const int x, const int y, const int depth, int Lim, const int last_dir)
{
    if (depth > Max)Tx = x, Ty = y, Max = depth;
    if (depth > Lim) return;
    Map[x][y] = '.';
    while (Check(x, y))
    {
        int t = RND() % 4;
        if (RND() % 1000 < RATE_KEEP_DIR) t = last_dir;
        int nx = x + dx[t], ny = y + dy[t], Nx = x + Dx[t], Ny = y + Dy[t];
        if (nx<0 || nx>n - 1 || ny<0 || ny>m - 1 || Map[nx][ny] != 'x')continue;
        if (Nx<0 || Nx>n - 1 || Ny<0 || Ny>m - 1 || Map[Nx][Ny] != 'x')continue;
        if (Nx == 0 || Nx == n - 1 || Ny == 0 || Ny == m - 1) {
            if ((int)(RND() % 1000) <= 75)  Map[nx][ny] = 'O';
            else if ((int)(RND() % 1000) >= 925) Map[nx][ny] = 'T';
            else Map[nx][ny] = '.';
            continue;
        }
        if ((int)(RND() % 1000) <= 75)  Map[nx][ny] = 'O';
        else if ((int)(RND() % 1000) >= 925) Map[nx][ny] = 'T';
        else { Map[nx][ny] = '.'; Map[Nx][Ny] = '.'; }
        Dfs(Nx, Ny, depth + 1, Lim, t);

        if ((int)(RND() % 1000) < (min(n, m) < 100 ? 0 : RETURN_RATE)) return;

        Lim = depth + max(min(n, m) / TWIST_RATE, 5);
    }
    return;
}

void saveMazeToFile(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing." << std::endl;
        return;
    }

    file << blocksize << '\n';
    file << m << '\n'; //col
    file << n << '\n'; //row

    for (int i = 0; i < n; ++i) for (int j = 0; j < m; ++j) Map[i][j] = 'x';
    Sx = 1, Sy = 1;
    Dfs(Sx, Sy, 0, max(min(n, m) / TWIST_RATE, 5), 2);
    //set starting and ending points.
    Map[Sx][Sy] = 'S';
    Map[Tx][Ty] = 'E';
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j)
            file << Map[i][j];
        file << '\n';
    }

    std::cout << "Maze saved to " << filename << std::endl;
    file.close();
}

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
    AIenemyMesh = renderer->LoadMesh("goose.msh");
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
    delete AIenemyMesh;

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

    if (player->GetTransform().GetPosition().y > 50.0f) {
        GameOneRunning(dt);
    }
    else if (player->GetTransform().GetPosition().y < 50.0f) {
        GameTwoRunning(dt);
    }

    renderer->Render();
    Debug::UpdateRenderables(dt);

    // Display main menu
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::ESCAPE)) {
        Window::GetWindow()->ShowOSPointer(true);
        Window::GetWindow()->LockMouseToWindow(false);
        Menu();
    }

    //cout << world->GetMainCamera().GetPosition() << endl;
}

void CourseWork::UpdateKeys() {
    if (Window::GetKeyboard()->KeyPressed(KeyCodes::Y)) { //switch Camera
        switchCamera = !switchCamera;
        if (switchCamera) { InitMainCamera(); }
        else if (!switchCamera) { InitPlayerCamera(); }
    }

    if (Window::GetKeyboard()->KeyPressed(KeyCodes::TAB)) { //switch Camera
        for (auto& i : keysFound)
        {
            cout << i.first << " " << i.second << endl;
        }
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
    keys.clear();
    enemies.clear();
    rotationBoard.clear();
    saveMazeToFile("../Assets/Data/maze.txt");
    grid = new NavigationGrid("maze.txt");
    world->ClearAndErase();
    physics->Clear();
    physics->UseGravity(useGravity);

    InitFloor();
    InitGameOneObject();

    InitPlayerCamera();

}

/*

A single function to add a large immoveable cube to the bottom of our world

*/
GameObject* CourseWork::AddFloorToWorld(const Vector3& position, float fSize) {
    GameObject* floor = new GameObject();

    Vector3 floorSize = Vector3(fSize, 1, fSize);
    AABBVolume* volume = new AABBVolume(floorSize);
    floor->SetBoundingVolume((CollisionVolume*)volume);
    floor->GetTransform()
        .SetScale(floorSize * 2)
        .SetPosition(Vector3(position.x + fSize, position.y, position.z + fSize));

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
    AABBVolume* volume = new AABBVolume(boardSize * 2.499f);
    Board->SetBoundingVolume((CollisionVolume*)volume);
    Board->GetTransform()
        .SetScale(boardSize * 4.999f)
        .SetPosition(position)
        .SetOrientation(Quaternion::AxisAngleToQuaterion(Vector3(1.0f, 0.0f, 0.0f), rotation.x) *
            Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 1.0f, 0.0f), rotation.y) *
            Quaternion::AxisAngleToQuaterion(Vector3(0.0f, 0.0f, 1.0f), rotation.z));

    Board->SetRenderObject(new RenderObject(&Board->GetTransform(), cubeMesh, basicTex, basicShader));
    //Board->GetRenderObject()->SetColour(Vector4(0.0, 0, 0.7, 1));
    Board->SetPhysicsObject(new PhysicsObject(&Board->GetTransform(), Board->GetBoundingVolume()));

    Board->GetPhysicsObject()->SetInverseMass(inverseMass);
    //Board->GetPhysicsObject()->InitCubeInertia();

    Board->SetUsesGravity(false);

    world->AddGameObject(Board);
    return Board;
}

GameObject* CourseWork::AddCapsuleToWorld(const Vector3& position, float meshSize, const std::string& name, float inverseMass) {

    GameObject* Capsule = new GameObject(name);

    CapsuleVolume* volume = new CapsuleVolume(meshSize, meshSize * 0.5f);
    Capsule->SetBoundingVolume((CollisionVolume*)volume);

    Capsule->GetTransform()
        .SetScale(Vector3(meshSize, meshSize, meshSize))
        .SetPosition(position);

    Capsule->SetRenderObject(new RenderObject(&Capsule->GetTransform(), capsuleMesh, basicTex, basicShader));
    Capsule->GetRenderObject()->SetColour(Vector4(1, 1, 0, 1));
    Capsule->SetPhysicsObject(new PhysicsObject(&Capsule->GetTransform(), Capsule->GetBoundingVolume()));

    Capsule->GetPhysicsObject()->SetInverseMass(inverseMass);
    //Capsule->GetPhysicsObject()->InitSphereInertia();

    world->AddGameObject(Capsule);

    return Capsule;
}

void CourseWork::InitFloor() {
    AddFloorToWorld(Vector3(-2.5f, 0, -2.5f), 50);
    AddFloorToWorld(Vector3(-2.5f, 110, -2.5f), 50);
}

void CourseWork::InitGameOneObject() {
    nodeSize = grid->GetNodeSize();
    grid->PrintAllNodes();
    //PartB area
    for (int y = 0; y < grid->GetGridHeight(); ++y) {
        for (int x = 0; x < grid->GetGridWidth(); ++x) {
            if (grid->GetAllNodes()[(grid->GetGridWidth() * y) + x].type == 'x')
                AddBoardToWorld(Vector3(nodeSize * x, nodeSize + 1, nodeSize * y),
                    Vector3(0, 0, 0), Vector3(1, 2, 1));
            else if (grid->GetAllNodes()[(grid->GetGridWidth() * y) + x].type == 'S')
                player = new Player(*this, Vector3(x * nodeSize, 2, y * nodeSize), charMesh, nullptr, basicShader);
            else if (grid->GetAllNodes()[(grid->GetGridWidth() * y) + x].type == 'E')
            {
                finalTreasurePos = Vector3(x * nodeSize, 0, y * nodeSize);
                /*keysPos.emplace_back(finalTreasurePos);*/
                finalTreasure = AddCapsuleToWorld(Vector3(x * nodeSize, 2, y * nodeSize), 1.0f, "key");
                finalTreasure->SetColour(Vector4(0, 0, 0, 1));
                keys.emplace_back(finalTreasure);
                aiEnemy = new AIEnemy(this, Vector3(x * nodeSize + 1.5, 4, y * nodeSize + 1.5),
                    AIenemyMesh, nullptr, basicShader, 0.5f);
            }
            else if (grid->GetAllNodes()[(grid->GetGridWidth() * y) + x].type == 'O')
            {
                rotationBoard.emplace_back(new RotationBoard(*this, Vector3(x * nodeSize, 5, y * nodeSize),
                    Vector3(0, 0, 0), Vector3(2.4f, 4.0f, 0.25f),
                    cubeMesh, basicTex, basicShader, 0.1f));
            }
            else if (grid->GetAllNodes()[(grid->GetGridWidth() * y) + x].type == 'T')
            {
                keys.emplace_back(AddCapsuleToWorld(Vector3(x * nodeSize, 2, y * nodeSize), 0.75f, "treasure"));
                keysPos.emplace_back(Vector3(x * nodeSize, 2, y * nodeSize));
            }
            else if (grid->GetAllNodes()[(grid->GetGridWidth() * y) + x].type == '.')
            {
                safePos.emplace_back(Vector3(x * nodeSize, 2, y * nodeSize));
            }
        }
    }
    InitKeysFound(keysPos);

    //PartA area
    AddBoardToWorld(Vector3(nodeSize, 100, nodeSize), Vector3(0, 0, 0), Vector3(1, 0.25, 1));
    //player = new Player(*this, Vector3(nodeSize, 98, nodeSize), charMesh, nullptr, basicShader);
    bridge = new Bridge(*this, Vector3(nodeSize * 2, 100, nodeSize), Vector3(2.5, 0.5, 2.5), 6, -1.0f);
    DamageLinkSphere = new  DamageObject(*this, Vector3(nodeSize * 5, 87.5, nodeSize), 1, 6, 6, -1.0f,
        sphereMesh, basicTex, basicShader);

    AddBoardToWorld(Vector3(nodeSize * 8, 100, nodeSize * 3), Vector3(0, 0, 0), Vector3(1, 0.25, 3));
    AddBoardToWorld(Vector3(nodeSize * 12, 100, nodeSize * 5.5), Vector3(0, 0, 0), Vector3(10, 0.25, 2));
    AddBoardToWorld(Vector3(nodeSize * 8, 100, nodeSize * 10.5), Vector3(0, 0, 0), Vector3(2, 0.25, 8));
    AddBoardToWorld(Vector3(nodeSize * 16, 100, nodeSize * 10.5), Vector3(0, 0, 0), Vector3(2, 0.25, 8));
    AddBoardToWorld(Vector3(nodeSize * 12, 100, nodeSize * 12), Vector3(0, 0, 0), Vector3(6, 0.25, 2));
    AddBoardToWorld(Vector3(nodeSize * 12, 100, nodeSize * 15), Vector3(0, 0, 0), Vector3(2, 0.25, 4));

    new SpringBoard(*this, Vector3(7.5 * nodeSize, 97, 3 * nodeSize),
        Vector3(0, 0, 0.0f), Vector3(0.15, 3, 7.75), cubeMesh, basicTex, basicShader);
    enemies.emplace_back(new Enemy(this, Vector3(7.5 * nodeSize, 97, 5.25 * nodeSize),
        enemyMesh, nullptr, basicShader, -0.5f, "h"));
    enemies.emplace_back(new Enemy(this, Vector3(7.75 * nodeSize, 97, 5.0 * nodeSize),
        enemyMesh, nullptr, basicShader, -0.5f, "v"));
    enemies.emplace_back(new Enemy(this, Vector3(16 * nodeSize, 97, 5.0 * nodeSize),
        enemyMesh, nullptr, basicShader, -0.5f, "v"));

    key = AddCapsuleToWorld(Vector3(12 * nodeSize, 96, nodeSize * 16), 1.0f, "key", -10.0f);
    key->SetColour(Vector4(0, 0, 0, 0.75f));

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

void CourseWork::ShowUIOne() {
    Debug::Print("Current Health: " + std::to_string(player->GetHealth()), Vector2(1.5, 5));
    return;
}

void CourseWork::ShowUITwo() {

    return;
}

//void CourseWork::PlayerUpdate(float dt) {
//    if (!switchCamera) {
//        Vector3 playerPos = player->GetTransform().GetPosition();
//        Vector3 linearImpulse;
//
//        //check isStand
//        Ray yRay = Ray(playerPos, Vector3(0, -1, 0));
//        RayCollision floorCollision;
//        if (world->Raycast(yRay, floorCollision, true, player))
//        {
//            float distance = (floorCollision.collidedAt - playerPos).Length();
//            distance <= 1.0f ? isStand = 1 : isStand = 0;
//        }
//
//        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::W)) {
//            linearImpulse.z = -1.0f;
//            //player->GetPhysicsObject()->AddForce(Vector3(0.0f, 0.0f, -10.0f));
//        }
//        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::S)) {
//            linearImpulse.z = 1.0f;
//        }
//        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::A)) {
//            linearImpulse.x = -1.0f;
//        }
//        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::D)) {
//            linearImpulse.x = 1.0f;
//        }
//        //calculate move forward angle
//        if (linearImpulse.Length() != 0.0f)
//        {
//            float newOrientation = RadiansToDegrees(atan2(-linearImpulse.x, -linearImpulse.z)) + world->GetMainCamera().GetYaw();
//            Quaternion newOrientationQ = Quaternion::EulerAnglesToQuaternion(0, newOrientation, 0);
//
//            player->GetTransform().SetOrientation(Quaternion::Slerp(player->GetTransform().GetOrientation(), newOrientationQ, 5 * dt));
//            player->GetPhysicsObject()->AddForce(newOrientationQ * Vector3(0, 0, -1.0f).Normalised() * 20);
//        }
//
//        if (Window::GetKeyboard()->KeyHeld(NCL::KeyCodes::LSHIFT)) {
//
//        }
//        if (Window::GetKeyboard()->KeyHeld(NCL::KeyCodes::LMENU)) {
//
//        }
//
//        // double jump
//        if (jumpCount >= 1 && isStand)
//        {
//            //clear elasticity
//            Vector3 playerLV = player->GetPhysicsObject()->GetLinearVelocity();
//            player->GetPhysicsObject()->SetLinearVelocity(Vector3(playerLV.x, 0, playerLV.z));
//            jumpCount = 0;
//        }
//        if (Window::GetKeyboard()->KeyPressed(NCL::KeyCodes::SPACE)) {
//            if (isStand) {
//                player->GetPhysicsObject()->ApplyLinearImpulse(Vector3(0, 1, 0) * 20);
//                ++jumpCount;
//            }
//            else if (jumpCount < 2) {
//                player->GetPhysicsObject()->ApplyLinearImpulse(Vector3(0, 1, 0) * 15);
//                ++jumpCount;
//            }
//        }
//        //cout << jumpCount << endl;
//
//        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::PLUS)) {
//
//        }
//        if (Window::GetKeyboard()->KeyDown(NCL::KeyCodes::MINUS)) {
//
//        }
//
//
//    }
//}

void CourseWork::GameOneRunning(float dt)
{
    ShowUIOne();
    //ShowUITwo();
    player->Update(dt);
    for (auto& i : enemies) i->Update(dt);
    //for (auto& i : rotationBoard) i->update();
    DamageLinkSphere->Update();


    world->GetMainCamera().UpdateCamera(dt);

    if (player->GetKey() && !springFlag) {
        springBoard = new SpringBoard(*this, Vector3(nodeSize, 99, nodeSize),
            Vector3(0, 0, 0.0f), Vector3(1, 0.25, 1), cubeMesh, basicTex, basicShader);
        springFlag = true;
    }
    else if (!player->GetKey() && springFlag) {
        world->RemoveGameObject(springBoard);
        springFlag = false;
    }

    //Debug::DrawLine(Vector3(), Vector3(0, 100, 0), Vector4(1, 0, 0, 1));

    /*SelectObject();
    MoveSelectedObject();*/
}

void CourseWork::GameTwoRunning(float dt)
{
    ShowUIOne();
    //ShowUITwo();
    Debug::DrawLine(finalTreasurePos, Vector3(finalTreasurePos.x, 30, finalTreasurePos.z), Debug::BLUE);
    player->Update(dt);
    aiEnemy->Update(dt);
    //for (auto& i : enemies) i->Update(dt);
    for (auto& i : rotationBoard) i->update();
    //DamageLinkSphere->Update();

    world->GetMainCamera().UpdateCamera(dt);
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
