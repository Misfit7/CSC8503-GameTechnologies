#pragma once
#include "Enemy.h"
class AIEnemy : public Enemy
{
public:

    void SetKeysPos(vector<Vector3> keysPos) {
        for (auto& i : keysPos) keysFound.emplace_back(pair<Vector3, bool>(i, false));
    };

protected:
    void Pathfinding(Vector3 startPos, Vector3 endPos);
    void DisplayPathfinding();

    //pathfind
    NavigationPath outPath;
    vector<Vector3> pathNodes;
    vector<pair<Vector3, bool>> keysFound;
    int currentKeyT = 0;
    int currentNodeT = 0;
};

