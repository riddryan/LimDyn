r = DynModel2D;
r.name = 'double_pendulum';

% Create Bodies & Joints
[r,shank] = r.addBody('shank','ground','hinge'); 
[r,thigh] = r.addBody('thigh','shank','hinge','d','lshanktothigh'*shank.xaxis);


% 
r = r.addSpring('shank','thigh','type','angular');
% r = r.addDamper('shank','thigh','type','angular');

% r = r.Build;