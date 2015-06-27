classdef DoublePendulum
%DESCRIPTION GOES HERE

properties

%Masses
mass1 = 1;
mass2 = 1;

%Inertias
inertia1 = 0;
inertia2 = 0;

%Distances from joint to COM
lcom1 = 1;
lcom2 = 1;

%Distances from previous COM to joint
lshanktothigh = 1;

%Springs
k1 = 1;
springlength1 = 1;
k2 = 1;
springlength2 = 1;

%Dampers
c1 = 1;
c2 = 1;

%World
g = -1;

%Spring Attachment Points
testl = 1;
end
end