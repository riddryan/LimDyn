r = DynModel2D;
r.name = 'double_pendulum';

% Create Bodies & Joints
[r,shank] = r.addBody('shank','ground','hinge'); 
[r,thigh] = r.addBody('thigh','shank','hinge','d','lshanktothigh'*shank.xaxis);


% 
r = r.addSpring('shank','thigh','type','angular');
r = r.addSpring('shank','thigh','type','linear','attachvec1','testl'*shank.xaxis);

r = r.addDamper('shank','thigh','type','angular');
r = r.addDamper('shank','thigh','type','linear');
r = r.Build;