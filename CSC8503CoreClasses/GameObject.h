#pragma once
#include "Transform.h"
#include "CollisionVolume.h"

using std::vector;
using namespace std;

namespace NCL::CSC8503 {
    class NetworkObject;
    class RenderObject;
    class PhysicsObject;

    enum class CollisionLayer {
        Default = 1,
        Wall = 2,
        Player = 4,
        Enemy = 8,
        Coins = 16,
        Floor = 32,
        UI = 256,
        Finish = 512
    };

    enum AxisLock {
        LINEAR_X = (1 << 0),
        LINEAR_Y = (1 << 1),
        LINEAR_Z = (1 << 2),
        ANGULAR_X = (1 << 3),
        ANGULAR_Y = (1 << 4),
        ANGULAR_Z = (1 << 5)
    };

    class GameObject {
    public:
        GameObject(const std::string& name = "");
        ~GameObject();

        void SetBoundingVolume(CollisionVolume* vol) {
            boundingVolume = vol;
        }

        const CollisionVolume* GetBoundingVolume() const {
            return boundingVolume;
        }

        bool IsActive() const {
            return isActive;
        }

        Transform& GetTransform() {
            return transform;
        }

        RenderObject* GetRenderObject() const {
            return renderObject;
        }

        PhysicsObject* GetPhysicsObject() const {
            return physicsObject;
        }

        NetworkObject* GetNetworkObject() const {
            return networkObject;
        }

        void SetRenderObject(RenderObject* newObject) {
            renderObject = newObject;
        }

        void SetPhysicsObject(PhysicsObject* newObject) {
            physicsObject = newObject;
        }

        const std::string& GetName() const {
            return name;
        }

        virtual void OnCollisionBegin(GameObject* otherObject) {
            //std::cout << "OnCollisionBegin event occured!\n";
        }

        virtual void OnCollisionEnd(GameObject* otherObject) {
            //std::cout << "OnCollisionEnd event occured!\n";
        }

        bool GetBroadphaseAABB(Vector3& outsize) const;

        void UpdateBroadphaseAABB();

        bool GetUsesGravity() const { return useGravity; }
        void SetUsesGravity(bool s) { useGravity = s; }

        void SetWorldID(int newID) {
            worldID = newID;
        }

        int		GetWorldID() const {
            return worldID;
        }

        void ConstrainLinearVelocity();
        void ConstrainAngularVelocity();

    protected:
        Transform			transform;

        CollisionVolume* boundingVolume;
        PhysicsObject* physicsObject;
        RenderObject* renderObject;
        NetworkObject* networkObject;

        bool		isActive;
        int			worldID;
        std::string	name;

        Vector3 broadphaseAABB;

        int lockFlags;
        bool useGravity;
    };
}

