#include "Window.h"

#include "Debug.h"

#include "StateMachine.h"
#include "StateTransition.h"
#include "State.h"

#include "GameServer.h"
#include "GameClient.h"

#include "NavigationGrid.h"
#include "NavigationMesh.h"

#include "CourseWork.h"
#include "NetworkedGame.h"

#include "PushdownMachine.h"

#include "PushdownState.h"

#include "BehaviourNode.h"
#include "BehaviourSelector.h"
#include "BehaviourSequence.h"
#include "BehaviourAction.h"

using namespace NCL;
using namespace CSC8503;

#include <chrono>
#include <thread>
#include <sstream>

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <ctime>

void TestBehaviourTree() {
    float behaviourTimer;
    float distanceToTarget;
    BehaviourAction* findKey = new BehaviourAction("Find Key",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {
                std::cout << "Looking for a key !\n";
                behaviourTimer = rand() % 100;
                state = Ongoing;
            }
            else if (state == Ongoing) {
                behaviourTimer -= dt;
                if (behaviourTimer <= 0.0f) {
                    std::cout << "Found a key !\n";
                    return Success;
                }
            }
            return state; // will be �� ongoing �� until success
        }
    );
    BehaviourAction* goToRoom = new BehaviourAction("Go To Room",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {
                std::cout << "Going to the loot room !\n";
                state = Ongoing;

            }
            else if (state == Ongoing) {
                distanceToTarget -= dt;
                if (distanceToTarget <= 0.0f) {
                    std::cout << "Reached room !\n";
                    return Success;
                }
            }
            return state; // will be �� ongoing �� until success
        }
    );
    BehaviourAction* openDoor = new BehaviourAction("Open Door",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {
                std::cout << "Opening Door !\n";
                return Success;
            }
            return state;
        }
    );
    BehaviourAction* lookForTreasure = new BehaviourAction(
        "Look For Treasure",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {
                std::cout << "Looking for treasure !\n";
                return Ongoing;

            }
            else if (state == Ongoing) {
                bool found = rand() % 2;
                if (found) {
                    std::cout << "I found some treasure !\n";
                    return Success;

                }
                std::cout << "No treasure in here ...\n";
                return Failure;
            }
            return state;
        }
    );
    BehaviourAction* lookForItems = new BehaviourAction(
        "Look For Items",
        [&](float dt, BehaviourState state) -> BehaviourState {
            if (state == Initialise) {
                std::cout << "Looking for items !\n";
                return Ongoing;

            }
            else if (state == Ongoing) {
                bool found = rand() % 2;
                if (found) {
                    std::cout << "I found some items !\n";
                    return Success;

                }
                std::cout << "No items in here ...\n";
                return Failure;

            }
            return state;
        }
    );

    BehaviourSequence* sequence =
        new BehaviourSequence("Room Sequence");
    sequence->AddChild(findKey);
    sequence->AddChild(goToRoom);
    sequence->AddChild(openDoor);

    BehaviourSelector* selection =
        new BehaviourSelector("Loot Selection");
    selection->AddChild(lookForTreasure);
    selection->AddChild(lookForItems);

    BehaviourSequence* rootSequence =
        new BehaviourSequence("Root Sequence");
    rootSequence->AddChild(sequence);
    rootSequence->AddChild(selection);

    for (int i = 0; i < 5; ++i) {
        rootSequence->Reset();
        behaviourTimer = 0.0f;
        distanceToTarget = rand() % 250;
        BehaviourState state = Ongoing;
        std::cout << "We �� re going on an adventure!\n";
        while (state == Ongoing) {
            state = rootSequence->Execute(1.0f); // fake dt
        }
        if (state == Success) {
            std::cout << "What a successful adventure!\n";
        }
        else if (state == Failure) {
            std::cout << "What a waste of time!\n";
        }
    }
    std::cout << "All done!\n";
}

class TestPacketReceiver : public PacketReceiver {
public:
    TestPacketReceiver(string name) {
        this->name = name;
    }
    void ReceivePacket(int type, GamePacket* payload, int source) {
        if (type == String_Message) {
            StringPacket* realPacket = (StringPacket*)payload;

            string msg = realPacket->GetStringFromData();

            std::cout << name << " received message: " << msg << std::endl;
        }
    }
protected:
    string name;
};

//void TestNetworking() {
//    NetworkBase::Initialise();
//
//    TestPacketReceiver serverReceiver("Server");
//    TestPacketReceiver clientReceiver("Client");
//
//    int port = NetworkBase::GetDefaultPort();
//
//    GameServer* server = new GameServer(port, 1);
//    GameClient* client = new GameClient();
//
//    server->RegisterPacketHandler(String_Message, &serverReceiver);
//    client->RegisterPacketHandler(String_Message, &clientReceiver);
//
//    bool canConnect = client->Connect(127, 0, 0, 1, port);
//
//    StringPacket* serverSP;
//    StringPacket* clientSP;
//    for (int i = 1; i <= 100; ++i) {
//        /*serverSP = new StringPacket("Server says hello!" + std::to_string(i));
//        server->SendGlobalPacket(*serverSP);
//
//        clientSP = new StringPacket("Client says hello!" + std::to_string(i));
//        client->SendPacket(*clientSP);*/
//
//        server->SendGlobalPacket(*(new StringPacket("Server says hello!" + std::to_string(i))));
//
//        client->SendPacket(*(new StringPacket("Client says hello!" + std::to_string(i))));
//
//        server->UpdateServer();
//        client->UpdateClient();
//
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//    }
//    NetworkBase::Destroy();
//}

/*

The main function should look pretty familar to you!
We make a window, and then go into a while loop that repeatedly
runs our 'game' until we press escape. Instead of making a 'renderer'
and updating it, we instead make a whole game, and repeatedly update that,
instead.

This time, we've added some extra functionality to the window class - we can
hide or show the

*/
int main() {
    /*TestBehaviourTree();
    TestNetworking();
    system("pause");*/

    Window* w = Window::CreateGameWindow("CSC8503 Game technology!", 1280, 720, false);

    if (!w->HasInitialised()) {
        return -1;
    }

    w->ShowOSPointer(false);
    w->LockMouseToWindow(true);

    NetworkedGame* g = new NetworkedGame();
    w->GetTimer().GetTimeDeltaSeconds(); //Clear the timer so we don't get a larget first dt!

    //TestPathfinding();

    while (w->UpdateWindow()) {
        float dt = w->GetTimer().GetTimeDeltaSeconds();
        if (dt > 0.1f) {
            std::cout << "Skipping large time delta" << std::endl;
            continue; //must have hit a breakpoint or something to have a 1 second frame time!
        }
        if (Window::GetKeyboard()->KeyPressed(KeyCodes::PRIOR)) {
            w->ShowConsole(true);
        }
        if (Window::GetKeyboard()->KeyPressed(KeyCodes::NEXT)) {
            w->ShowConsole(false);
        }

        if (Window::GetKeyboard()->KeyPressed(KeyCodes::T)) {
            w->SetWindowPosition(0, 0);
        }

        w->SetTitle("Gametech frame time:" + std::to_string(1000.0f * dt));

        g->UpdateGame(dt);

        //DisplayPathfinding();
    }
    Window::DestroyGameWindow();
}