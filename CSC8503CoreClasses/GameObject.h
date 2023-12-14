#pragma once
#include "Transform.h"
#include "CollisionVolume.h"
#include "RenderObject.h"
#include "PhysicsObject.h"

using std::vector;
using namespace std;

namespace NCL {
    namespace CSC8503 {
        class NetworkObject;
        class RenderObject;
        class PhysicsObject;

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

            int	GetWorldID() const {
                return worldID;
            }

            void ConstraintVelocity(bool isLinear) {
                Vector3 velocity = isLinear ? physicsObject->GetLinearVelocity() : physicsObject->GetAngularVelocity();

                if (lockFlags & LINEAR_X || lockFlags & ANGULAR_X) {
                    velocity.x = 0;
                }
                if (lockFlags & LINEAR_Y || lockFlags & ANGULAR_Y) {
                    velocity.y = 0;
                }
                if (lockFlags & LINEAR_Z || lockFlags & ANGULAR_Z) {
                    velocity.z = 0;
                }

                isLinear ? physicsObject->SetLinearVelocity(velocity) : physicsObject->SetAngularVelocity(velocity);
            }

            void SetLockFlags(int lf) {
                lockFlags = lf;
            }

            void SetColour(Vector4 c) {
                colour = c;
                if (renderObject) {
                    renderObject->SetColour(colour);
                }
            }

            Vector4	GetColour() const {
                return colour;
            }

        protected:
            Transform			transform;

            CollisionVolume* boundingVolume;
            PhysicsObject* physicsObject;
            RenderObject* renderObject;
            NetworkObject* networkObject;

            bool		isActive;
            int			worldID;
            std::string	name;
            //map<>

            Vector3 broadphaseAABB;

            Vector4 colour;

            int lockFlags;
            bool useGravity;
        };
    }
}