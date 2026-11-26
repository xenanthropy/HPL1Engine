/* HPL1Engine addition to Julio Jerez's Newton 1.53 compatibility layer.
* Same zlib licence as the rest of the layer (see LICENSE). Never sent upstream.
*
* In 1.53, a hinge, slider or corkscrew user callback can ask for one extra
* constraint row along the joint's pin: it fills in a NewtonHingeSliderUpdateDesc
* (an acceleration and optional force bounds) and returns non-zero. HPL1 builds
* all its joint limits (doors, levers, drawers, screws), "friction" and motors
* on this. The layer called the callbacks but threw the result away; these
* helpers add the row the way Newton 2.36 (the closest open relative of 1.53,
* dgHingeConstraint / dgSlidingConstraint / dgCorkscrewConstraint) does:
* after the joint's base rows, along the child's pin axis, as a motor row with
* the requested acceleration, bounded by the requested min/max force.
*/

#ifndef NEWTON_HPL1_JOINTS_H
#define NEWTON_HPL1_JOINTS_H

#include "newtonStdafx.h"
#include "Newton.h"

// Unbounded, as in 1.53: a callback that leaves these alone gets a row with no
// force limit. HPL1 sets one side to 0 to make a one-sided stop.
inline void HPL1InitCallbackDesc(NewtonHingeSliderUpdateDesc& param, const ndConstraintDescritor& desc)
{
	param.m_accel = dFloat(0.0f);
	param.m_timestep = desc.m_timestep;
	param.m_minFriction = D_MIN_BOUND;
	param.m_maxFriction = D_MAX_BOUND;
}

// Adds the row a callback asked for. angular = rotation about the pin (hinge,
// corkscrew rotation); otherwise translation along the pin (slider, corkscrew
// translation).
inline void HPL1SubmitCallbackRow(ndJointBilateralConstraint* const joint, ndConstraintDescritor& desc, const NewtonHingeSliderUpdateDesc& param, bool angular)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	joint->CalculateGlobalMatrix(matrix0, matrix1);
	if (angular)
	{
		joint->AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
	}
	else
	{
		// The point on the parent's pin line closest to the child's pivot, so
		// the row carries no position error of its own (as 2.36 does).
		const ndFloat32 along = (matrix0.m_posit - matrix1.m_posit).DotProduct(matrix1.m_front).GetScalar();
		const ndVector p1(matrix1.m_posit + matrix1.m_front.Scale(along));
		joint->AddLinearRowJacobian(desc, matrix0.m_posit, p1, matrix0.m_front);
	}
	joint->SetMotorAcceleration(desc, param.m_accel);
	if (param.m_minFriction > D_MIN_BOUND)
	{
		joint->SetLowerFriction(desc, param.m_minFriction);
	}
	if (param.m_maxFriction < D_MAX_BOUND)
	{
		joint->SetHighFriction(desc, param.m_maxFriction);
	}
}

#endif
