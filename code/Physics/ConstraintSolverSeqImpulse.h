#ifndef CONSTRAINT_SOLVER_SEQ_IMPULSE_H
#define CONSTRAINT_SOLVER_SEQ_IMPULSE_H

#include "IConstraintSolver.h"

class ConstraintSolverSeqImpulse : public IConstraintSolver
{
public:

	void SolveConstraints(std::vector<PhysicsDefs::CollPairContactInfo>& info, float dt) override;

};

#endif