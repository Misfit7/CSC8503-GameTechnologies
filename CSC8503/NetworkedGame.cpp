#include "NetworkedGame.h"
#include "NetworkPlayer.h"
#include "NetworkObject.h"
#include "GameServer.h"
#include "GameClient.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include "Assets.h"

using namespace NCL;
using namespace CSC8503;

using namespace std;

#define COLLISION_MSG 30

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
    std::ofstream file(Assets::DATADIR + filename);
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

struct MessagePacket : public GamePacket {
    short playerID;
    short messageID;

    MessagePacket() {
        type = Message;
        size = sizeof(short) * 2;
    }
};

NetworkedGame::NetworkedGame() {
    thisServer = nullptr;
    thisClient = nullptr;

    NetworkBase::Initialise();
    timeToNextPacket = 0.0f;
    packetsToSnapshot = 0;

    Menu();
}

NetworkedGame::~NetworkedGame() {
    delete thisServer;
    delete thisClient;
}

void NetworkedGame::StartAsServer() {
    this->name = "Server";
    thisServer = new GameServer(NetworkBase::GetDefaultPort(), 2);

    thisServer->RegisterPacketHandler(Received_State, this);
    thisServer->RegisterPacketHandler(String_Message, this);
}

void NetworkedGame::StartAsClient(char a, char b, char c, char d) {
    this->name = "Client";
    thisClient = new GameClient();
    thisClient->Connect(a, b, c, d, NetworkBase::GetDefaultPort());

    thisClient->RegisterPacketHandler(Delta_State, this);
    thisClient->RegisterPacketHandler(Full_State, this);
    thisClient->RegisterPacketHandler(Player_Connected, this);
    thisClient->RegisterPacketHandler(Player_Disconnected, this);
    thisClient->RegisterPacketHandler(String_Message, this);
}

void NetworkedGame::UpdateGame(float dt) {
    timeToNextPacket -= dt;
    if (timeToNextPacket < 0) {
        if (thisServer) {
            UpdateAsServer(dt);
            thisServer->UpdateServer();
            UpdateAsClient(dt);
            thisClient->UpdateClient();
        }
        else if (thisClient) {
            UpdateAsClient(dt);
            thisClient->UpdateClient();
        }
        timeToNextPacket += 1.0f / 20.0f; //20hz server/client update
    }

    if (localPlayer) {
        renderer->Render();
        Debug::UpdateRenderables(dt);
        // Display main menu
        if (Window::GetKeyboard()->KeyPressed(KeyCodes::ESCAPE)) {
            Window::GetWindow()->ShowOSPointer(true);
            Window::GetWindow()->LockMouseToWindow(false);
            Menu();
        }

        if (gameTime < 0.0f) {
            currentGame = 0;
            Menu();
        }
        else if ((player->GetTransform().GetPosition() - Vector3(5, 0, 5)).Length() <= 2.5f && player->GetKey())
        {
            currentGame = 0;
            Menu();
        }
        //cout << world->GetMainCamera().GetPosition() << endl;

        //UpdateMinimumState();

        CourseWork::UpdateGame(dt);
        //player update
        for (const auto& pair : serverPlayers) {
            pair.second->Update(dt);
        }
        for (const auto& pair : scores) {
            Debug::Print("Player" + std::to_string(pair.first) + ":" + std::to_string(pair.second), Vector2(80, 20 + pair.first * 5));
        }
    }
}

void NetworkedGame::Menu(const std::string& text, const Vector4& colour) {
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

void NetworkedGame::SetGameState(int value) {
    if (value == 1) {
        cout << "StartAsServer" << endl;
        if (!thisServer && !thisClient) {
            saveMazeToFile("maze.txt");
            InitGameOne();
            StartAsServer();
            StartAsClient(127, 0, 0, 1);
        }
        else {
            InitGameOne();
            player = new Player(*this, Vector3(nodeSize, 98, nodeSize),
                charMesh, nullptr, basicShader, 0, "player");
            localPlayer = player;
            InitPlayerCamera();
            serverPlayers[localplayerID] = player;
        }
    }
    else if (value == 2) {
        cout << "StartAsClient" << endl;
        if (!thisClient) {
            InitGameOne();
            thisServer = nullptr;
            StartAsClient(127, 0, 0, 1);
        }
        else {
            InitGameOne();
            player = new Player(*this, Vector3(nodeSize, 98, nodeSize),
                charMesh, nullptr, basicShader, 0, "player");
            localPlayer = player;
            InitPlayerCamera();
            serverPlayers[localplayerID] = player;
        }
    }
    else if (value == 3) {
        Window::DestroyGameWindow();
        exit(0);
    }
    else if (value == 4) {
        InitGameOne();
        player = new Player(*this, Vector3(nodeSize, 98, nodeSize),
            charMesh, nullptr, basicShader, 0, "player");
        localPlayer = player;
        InitPlayerCamera();
        serverPlayers[localplayerID] = player;
    }
}

void NetworkedGame::UpdateAsServer(float dt) {
    for (const auto& pair : scores) {
        GamePacket* score = new StringPacket(to_string(pair.first) + " " + to_string(pair.second));
        thisServer->SendGlobalPacket(*score);
    }

    packetsToSnapshot--;
    if (packetsToSnapshot < 0) {
        BroadcastSnapshot(false);
        packetsToSnapshot = 5;
    }
    else {
        BroadcastSnapshot(true);
    }
}

void NetworkedGame::UpdateAsClient(float dt) {
    GamePacket* score = new StringPacket(to_string(localplayerID) + " " + to_string(playerscore));

    ////if (Window::GetKeyboard()->KeyPressed(KeyCodes::SPACE)) {
    ////    //fire button pressed!
    ////    newPacket.buttonstates[0] = 1;
    ////    newPacket.lastID = 0; //You'll need to work this out somehow...
    ////}
    thisClient->SendPacket(*score);
}

Player* NetworkedGame::SpawnPlayer(int playerNum)
{
    return new Player(*this, Vector3(nodeSize, 98, nodeSize), charMesh, nullptr, basicShader, playerNum, "player");
}

void NetworkedGame::BroadcastSnapshot(bool deltaFrame) {
    std::vector<GameObject*>::const_iterator first;
    std::vector<GameObject*>::const_iterator last;

    world->GetObjectIterators(first, last);

    for (auto i = first; i != last; ++i) {
        NetworkObject* o = (*i)->GetNetworkObject();
        if (!o) {
            continue;
        }
        //TODO - you'll need some way of determining
        //when a player has sent the server an acknowledgement
        //and store the lastID somewhere. A map between player
        //and an int could work, or it could be part of a 
        //NetworkPlayer struct. 
        int playerState = 0;
        GamePacket* newPacket = nullptr;
        if (o->WritePacket(&newPacket, deltaFrame, playerState)) {
            thisServer->SendGlobalPacket(*newPacket);
            delete newPacket;
        }
    }
}
/*GamePacket* stringPacket = new StringPacket("Server says hello!");
    thisServer->SendGlobalPacket(*stringPacket);
    delete stringPacket;*/

void NetworkedGame::UpdateMinimumState() {
    //Periodically remove old data from the server
    int minID = INT_MAX;
    int maxID = 0; //we could use this to see if a player is lagging behind?

    for (auto i : stateIDs) {
        minID = std::min(minID, i.second);
        maxID = std::max(maxID, i.second);
    }
    //every client has acknowledged reaching at least state minID
    //so we can get rid of any old states!
    std::vector<GameObject*>::const_iterator first;
    std::vector<GameObject*>::const_iterator last;
    world->GetObjectIterators(first, last);

    for (auto i = first; i != last; ++i) {
        NetworkObject* o = (*i)->GetNetworkObject();
        if (!o) {
            continue;
        }
        o->UpdateStateHistory(minID); //clear out old states so they arent taking up memory...
    }
}

void NetworkedGame::ReceivePacket(int type, GamePacket* payload, int source) {
    if (type == String_Message) {
        StringPacket* realPacket = (StringPacket*)payload;

        string msg = realPacket->GetStringFromData();

        std::istringstream iss(msg);
        int playerID, playerscore;
        if (iss >> playerID >> playerscore) {
            scores[playerID] = max(scores[playerID], playerscore);
        }
        //std::cout << name << " received message: " << msg << std::endl;
    }
    else if (type == Player_Connected) {
        AddPlayerPacket* realPacket = (AddPlayerPacket*)payload;
        std::cout << name << " add player: " << realPacket->playerID << std::endl;
        if (!localPlayer) {
            player = new Player(*this, Vector3(nodeSize, 98, nodeSize),
                charMesh, nullptr, basicShader, 0, "player");
            localPlayer = player;
            localplayerID = realPacket->playerID;
            InitPlayerCamera();
            serverPlayers[realPacket->playerID] = player;
        }
        else {
            Player* newplayer = SpawnPlayer(realPacket->playerID);
            serverPlayers[realPacket->playerID] = newplayer;
        }
    }
    else if (type == Player_Disconnected) {
        DeletePlayerPacket* realPacket = (DeletePlayerPacket*)payload;
        std::cout << name << " delete player: " << realPacket->playerID << std::endl;
        world->RemoveGameObject(serverPlayers[realPacket->playerID]);
        serverPlayers.erase(realPacket->playerID);
        scores.erase(realPacket->playerID);
    }
}

void NetworkedGame::OnPlayerCollision(NetworkPlayer* a, NetworkPlayer* b) {
    if (thisServer) { //detected a collision between players!
        MessagePacket newPacket;
        newPacket.messageID = COLLISION_MSG;
        newPacket.playerID = a->GetPlayerNum();

        thisClient->SendPacket(newPacket);

        newPacket.playerID = b->GetPlayerNum();
        thisClient->SendPacket(newPacket);
    }
}