#ifndef CONSTRAINT_SOLVER_SEQ_IMPULSE_H
#define CONSTRAINT_SOLVER_SEQ_IMPULSE_H

#include "IConstraintSolver.h"

class ConstraintSolverSeqImpulse : public IConstraintSolver
{
public:
	void SolveConstraints(std::vector<PhysicsDefs::CollPairContactInfo>& info, float dt) override;
	void SolveConstraints2(std::vector<Manifold>& info, float dt) override;

private:
	void PreStep(std::vector<Manifold>& info, float dt);
	void SolveContact(IRigidBody* body1, IRigidBody* body2, PhysicsDefs::ContactInfo& info, float dt);

};

#endif