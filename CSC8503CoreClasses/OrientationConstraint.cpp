#include "OrientationConstraint.h"
#include "GameObject.h"
#include "PhysicsObject.h"
using namespace NCL;
using namespace Maths;
using namespace CSC8503;

OrientationConstraint::OrientationConstraint(GameObject* a, GameObject* b, float angle)
{
	objectA = a;
	objectB = b;
	maxRotationRadians = angle;
}

OrientationConstraint::~OrientationConstraint()
{

}

void OrientationConstraint::UpdateConstraint(float dt) {
	Quaternion qA = objectA->GetTransform().GetOrientation();
	Quaternion qB = objectB->GetTransform().GetOrientation();
	PhysicsObject* physA = objectA->GetPhysicsObject();
	PhysicsObject* physB = objectB->GetPhysicsObject();
	float invMassA = physA->GetInverseMass();
	float invMassB = physB->GetInverseMass();
	float constraintMass = invMassA + invMassB;
	//计算相对旋转
	Quaternion relativeRotation = qB.Conjugate() * qA;
	relativeRotation.Normalise();
	//计算旋转角度
	float angle = 2.0f * acos(std::clamp(relativeRotation.w, -1.0f, 1.0f));
	if (angle > maxRotationRadians) {
		float correctionFactor = maxRotationRadians / angle;
		//修正旋转到最大允许角度
		Quaternion correction = Quaternion::Lerp(Quaternion(), relativeRotation, correctionFactor);
		correction.Normalise();

		Vector3 correctionAxis = Vector3(correction.x, correction.y, correction.z);
		if (Vector::Length(correctionAxis) > 0.0f) {
			correctionAxis = Vector::Normalise(correctionAxis);
			float correctionAngle = angle - maxRotationRadians;
			// 根据惯性计算角动量修正

			if (constraintMass > 0.0f) {
				Vector3 angularImpulse = correctionAxis * (correctionAngle / constraintMass);
				physA->ApplyAngularImpulse(-angularImpulse * invMassA);
				physB->ApplyAngularImpulse(angularImpulse * invMassB);
			}
		}
	}
}