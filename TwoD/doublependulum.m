r = DynModel2D;
r.name = 'double_pendulum';

r = r.addBody('bodyname','shank');


r = r.addBody('bodyname','thigh');

r = r.addDOF('shank','ground','hinge');
r = r.addDOF('thigh','shank','hinge');

r = r.addSpring('shank','thigh','type','angular');
r = r.addDamper('shank','thigh','type','angular');

r = r.Build;