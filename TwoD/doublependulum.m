
r = DynModel2D;
r.name = 'double_pendulum';

r = r.addBody('bodyname','shank');
r = r.addBody('bodyname','thigh');

r = r.addJoint('shank','ground','hinge');
r = r.addJoint('thigh','shank','hinge');

r = r.addSpring('shank','thigh','type','angular');
r = r.addDamper('shank','thigh','type','angular');


r = r.getConstraintMatrix;
r = r.getConstraintMatrixDot;
r = r.getMassMatrix;
r = r.getGravityForces;
r = r.getExForces;

