#pragma once

using std::vector;

using namespace NCL::Maths;

namespace NCL {
    namespace CSC8503 {
        class Transform
        {
        public:
            Transform();
            ~Transform();

            Transform& SetPosition(const Vector3& worldPos);
            Transform& SetScale(const Vector3& worldScale);
            Transform& SetOrientation(const Quaternion& newOr);

            Vector3 GetPosition() const {
                return position;
            }

            Vector3 GetScale() const {
                return scale;
            }

            Quaternion GetOrientation() const {
                return orientation;
            }

            Matrix4 GetMatrix() const {
                return matrix;
            }

            Vector3 GetForward()
            {
                Matrix4 local = matrix;
                local.SetPositionVector(Vector3(0, 0, 0));
                Vector3 forward = local * Vector4(0.0f, 0.0f, -1.0f, 1.0f);
                return forward;
            }

            Vector3 GetUp()
            {
                Matrix4 local = matrix;
                local.SetPositionVector(Vector3(0, 0, 0));
                Vector3 up = local * Vector4(0.0f, 1.0f, 0.0f, 1.0f);
                return up;
            }

            Vector3 GetRight()
            {
                Matrix4 local = matrix;
                local.SetPositionVector(Vector3(0, 0, 0));
                Vector3 right = local * Vector4(1.0f, 0.0f, 0.0f, 1.0f);
                return right;
            }

            void UpdateMatrix();
        protected:
            Matrix4		matrix;
            Quaternion	orientation;
            Vector3		position;

            Vector3		scale;
        };
    }
}

