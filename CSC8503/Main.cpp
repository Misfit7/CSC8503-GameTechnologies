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

vector <Vector3> testNodes;
void TestPathfinding() {
    NavigationGrid grid("TestGrid1.txt");

    NavigationPath outPath;

    Vector3 startPos(80, 0, 10);
    Vector3 endPos(80, 0, 80);

    bool found = grid.FindPath(startPos, endPos, outPath);

    Vector3 pos;
    while (outPath.PopWaypoint(pos)) {
        testNodes.push_back(pos);
    }
}

void DisplayPathfinding() {
    for (int i = 1; i < testNodes.size(); ++i) {
        Vector3 a = testNodes[i - 1];
        Vector3 b = testNodes[i];

        Debug::DrawLine(a, b, Vector4(0, 1, 0, 1));
    }
}

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
    Window* w = Window::CreateGameWindow("CSC8503 Game technology!", 1280, 720, false);

    if (!w->HasInitialised()) {
        return -1;
    }

    w->ShowOSPointer(false);
    w->LockMouseToWindow(true);

    CourseWork* g = new CourseWork();
    w->GetTimer().GetTimeDeltaSeconds(); //Clear the timer so we don't get a larget first dt!

    TestPathfinding();

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

        DisplayPathfinding();
    }
    Window::DestroyGameWindow();
}