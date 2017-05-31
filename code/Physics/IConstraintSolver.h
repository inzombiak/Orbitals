#ifndef I_CONSTRAINT_SOLVER_H
#define I_CONSTRAINT_SOLVER_H

#include "IRigidBody.h"
#include "Manifold.h"

class IConstraintSolver
{
public:
	virtual ~IConstraintSolver() {}

	virtual void SolveConstraints(std::vector<PhysicsDefs::CollPairContactInfo>& info, float dt) = 0;
	virtual void SolveConstraints2(std::vector<Manifold>& info, float dt) = 0;
};

#endif