
r = DynModel2D;
r.name = 'double_pendulum';

r = r.addBody('bodyname','shank');
r = r.addBody('bodyname','thigh');

r = r.addJoint('shank','ground','hinge');
r = r.addJoint('thigh','shank','hinge');

r = r.getConstraintMatrix;
r = r.getConstraintMatrixDot;
r = r.getMassMatrix;
r = r.getGravityForces;

