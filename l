[33mcommit a8e09c945ab1cffa315303b10c5aaa73a099591a[m
Author: Ryan Riddick <ryridry@gmail.com>
Date:   Thu Mar 12 19:21:14 2015 -0400

    Separate 2D & 3D Model Files

[1mdiff --git a/DynModel.m b/DynModel.m[m
[1mdeleted file mode 100644[m
[1mindex 92cd368..0000000[m
[1m--- a/DynModel.m[m
[1m+++ /dev/null[m
[36m@@ -1,82 +0,0 @@[m
[31m-classdef DynModel[m
[31m-    [m
[31m-    properties[m
[31m-        name = 'Body1';[m
[31m-        mass = 1;[m
[31m-        inertia = 0;[m
[31m-        length = sym('l1');[m
[31m-        compos = 0.5;[m
[31m-        DOF = [0 0 1];[m
[31m-        axes = [1 1];[m
[31m-        endpoint = sym([0;0]);[m
[31m-        bodynumber = 0;[m
[31m-        [m
[31m-    end[m
[31m-    [m
[31m-    methods (Static)[m
[31m-       [m
[31m-        function [ground] = ground()[m
[31m-           ground = body2d;[m
[31m-           ground.mass = 0;[m
[31m-           ground.length = 0;[m
[31m-           ground.compos = 0;[m
[31m-           ground.DOF = [0 0 0];[m
[31m-        end[m
[31m-        [m
[31m-    end[m
[31m-    [m
[31m-    [m
[31m-    methods[m
[31m-        [m
[31m-        function [this] = body2d(this,prevBody,varargin)[m
[31m-            [m
[31m-            hingeaxis = [];[m
[31m-            RelativeTo = 'ground';[m
[31m-            slideraxis = [];[m
[31m-            [m
[31m-            for i = 1 : 2 : length(varargin)[m
[31m-                option = varargin{i};[m
[31m-                value = varargin{i + 1};[m
[31m-                switch option[m
[31m-                    case 'Hinge'[m
[31m-                        hingeaxis = val;[m
[31m-                    case 'RelativeTo'[m
[31m-                        RelativeTo = val;[m
[31m-                    case 'Slider'[m
[31m-                        slideraxis = val;[m
[31m-                        [m
[31m-                end[m
[31m-            end[m
[31m-            [m
[31m-            [m
[31m-            if ~isempty(hingeaxis) && ~isempty(slideraxis)[m
[31m-               error('You can''t define both a hinge and a slider for a new body') [m
[31m-            end[m
[31m-            [m
[31m-            bodynumber = prevbody.bodynumber + 1;[m
[31m-            this.bodynumber = bodynumber;[m
[31m-            [m
[31m-            for i = 1:bodynumber[m
[31m-                name = sprintf('q%d',i);[m
[31m-                eval([sprintf('q%d',i) ' = sym(name);']);[m
[31m-                uname = sprintf('u%d',i);[m
[31m-                eval([sprintf('u%d',i) ' = sym(uname);']);[m
[31m-                [m
[31m-                qs(i)=eval(sprintf('q%d',i));[m
[31m-                us(i)=eval(sprintf('u%d',i));[m
[31m-            end[m
[31m-            [m
[31m-            if strcmp(prevBody,'ground')[m
[31m-                prevBody  = body2d.ground;[m
[31m-            end[m
[31m-            [m
[31m-            if ~isempty(hingeaxis)[m
[31m-                angle = qs(bodynumber);[m
[31m-                this.endpoint = prevbody.endpoint + this.length*[cos(angle);sin(angle)];[m
[31m-                [m
[31m-            end[m
[31m-            [m
[31m-        end[m
[31m-    end[m
[31m-    [m
[31m-end[m
\ No newline at end of file[m
[1mdiff --git a/DynModel2D.m b/DynModel2D.m[m
[1mdeleted file mode 100644[m
[1mindex 9a175fb..0000000[m
[1m--- a/DynModel2D.m[m
[1m+++ /dev/null[m
[36m@@ -1,545 +0,0 @@[m
[31m-classdef DynModel2D[m
[31m-    [m
[31m-    properties[m
[31m-        name = 'Model2D_1';[m
[31m-        bodies = body2d.empty;[m
[31m-        joints = struct('constrainedbody',cell(1),'relativebody',cell(1),...[m
[31m-                        'joint',cell(1),'angleoffset',0,'numjoints',0); %joint info structure[m
[31m-        springs = struct('body1',cell(1),'body2',cell(1),'type',cell(1),'name',cell(1),'restlength',cell(1),'numsprings',0);[m
[31m-        dampers = struct('body1',cell(1),'body2',cell(1),'type',cell(1),'name',cell(1),'numdampers',0);[m
[31m-        groundslopeangle = sym(0);[m
[31m-        M = sym([]); %Maximal Mass Matrix[m
[31m-        C = sym([]); %Constraint Matrix[m
[31m-        Cdot = sym([]); %Derivative of constraint matrix[m
[31m-        G = sym([]); %Forces due to gravity[m
[31m-        ExForces = sym([]); %External Forces (springs, dampers, etc.)[m
[31m-    end[m
[31m-    [m
[31m-    properties (Dependent = true)[m
[31m-        numbodies %number of rigid bodies[m
[31m-        gravity[m
[31m-        dof[m
[31m-        posdexes[m
[31m-        veldexes[m
[31m-    end[m
[31m-    [m
[31m-    [m
[31m-    [m
[31m-    methods[m
[31m-        [m
[31m-        %% Access Methods[m
[31m-        function numbodies = get.numbodies(this)[m
[31m-           numbodies = length(this.bodies);[m
[31m-        end[m
[31m-        [m
[31m-        function dof = get.dof(this)[m
[31m-            dof = 3*this.numbodies;[m
[31m-        end[m
[31m-        [m
[31m-        function posdexes = get.posdexes(this)[m
[31m-           posdexes = 1:(this.dof/2); [m
[31m-        end[m
[31m-        [m
[31m-        function veldexes = get.veldexes(this)[m
[31m-            veldexes = (this.dof/2+1):this.dof;[m
[31m-        end[m
[31m-         [m
[31m-        function this = set.bodies(this,newbodies)[m
[31m-            [m
[31m-            if ~isa(this.bodies,'body2d')[m
[31m-               error('this.bodies must be of class "body2d"') [m
[31m-            end[m
[31m-    [m
[31m-            for i = 1:length(newbodies)[m
[31m-               names{i} = newbodies(i).bodyname;[m
[31m-            end[m
[31m-            uniquenames = unique(names);[m
[31m-            [m
[31m-            if length(uniquenames)<length(newbodies)[m
[31m-               error('You must assign a new name to each body in the model')[m
[31m-            else[m
[31m-                this.bodies = newbodies;[m
[31m-            end[m
[31m-        end[m
[31m-        [m
[31m-        function gravity = get.gravity(this)[m
[31m-            gravity = sym([0;-1]);[m
[31m-            alpha = this.groundslopeangle;[m
[31m-            Rotation = [cos(alpha) sin(alpha); -sin(alpha) cos(alpha)];[m
[31m-            gravity = Rotation*gravity;[m
[31m-        end[m
[31m-        [m
[31m-        %% User-Input Model Specification[m
[31m-        [m
[31m-        function [this] = addBody(this,varargin)[m
[31m-            %Creates a new bodye.  You can set[m
[31m-            %its properties bodyname, mass, inertia, length (d), and lcom.            [m
[31m-            mass = sym([]); inertia = sym([]); d = sym([]); lcom = sym([]); bodyname = char([]);[m
[31m-            for i = 1 : 2 : length(varargin)[m
[31m-                option = varargin{i};[m
[31m-                val = varargin{i + 1};[m
[31m-                switch option[m
[31m-                    case 'mass'[m
[31m-                        mass = val;[m
[31m-                    case 'inertia'[m
[31m-                        inertia = val;[m
[31m-                    case 'd'[m
[31m-                        d = val;[m
[31m-                    case 'lcom'[m
[31m-                        lcom = val;[m
[31m-                    case 'bodyname'[m
[31m-                        bodyname = val;[m
[31m-                end[m
[31m-            end[m
[31m-            [m
[31m-            Body = body2d;[m
[31m-            BodyPropNames = fieldnames(Body.bodyprops);[m
[31m-            oldnumberofbodies = this.numbodies;[m
[31m-            [m
[31m-            %Assign dynamic properties of the new body[m
[31m-            for i = 1:length(BodyPropNames)[m
[31m-               %Assign the property to user input value[m
[31m-               Body.(BodyPropNames{i}) =  eval(BodyPropNames{i});[m
[31m-               [m
[31m-               %If user input no value, give it a default value based on[m
[31m-               %the number of the body that is[m
[31m-               if isempty(Body.(BodyPropNames{i}))[m
[31m-                   Body.(BodyPropNames{i}) = eval( sprintf('sym('' %s%d '')', BodyPropNames{i}, oldnumberofbodies+1) );[m
[31m-               end[m
[31m-                [m
[31m-            end[m
[31m-            [m
[31m-[m
[31m-            %If user input no value, give it name BodyX, where X is the[m
[31m-            %number of the new body[m
[31m-            if isempty(bodyname)[m
[31m-                Body.bodyname = eval( sprintf('Body%d', oldnumberofbodies+1) );[m
[31m-            else %Assign body name based on user input[m
[31m-                 Body.bodyname = bodyname;[m
[31m-            end[m
[31m-            [m
[31m-            if oldnumberofbodies~=0[m
[31m-                this.bodies = [this.bodies Body];[m
[31m-            else[m
[31m-                this.bodies = Body;[m
[31m-            end[m
[31m-            [m
[31m-        end[m
[31m-        [m
[31m-        function [this] = addJoint(this,constrainedbodyname,relativebodyname,joint,varargin)[m
[31m-            %Stores the information of a new joint "joint" relative to the body2d[m
[31m-            %"relativebody" in this.joints[m
[31m-            [m
[31m-            %angleoffset only pertains to joints "fixed" and "slider".  The[m
[31m-            %value of angle offset determines the CCW rotational offset of[m
[31m-            %constrainedbody relative to relativebody.[m
[31m-            [m
[31m-            angleoffset = 0; %only for fixed and slider joints[m
[31m-            [m
[31m-            for i = 1 : 2 : length(varargin)[m
[31m-                option = varargin{i};[m
[31m-                val = varargin{i + 1};[m
[31m-                switch option[m
[31m-                    case 'angleoffset'[m
[31m-                        angleoffset = val;[m
[31m-                end[m
[31m-            end[m
[31m-            [m
[31m-            if strcmp(joint,'Hinge')[m
[31m-               angleoffset = 0; [m
[31m-            end[m
[31m-            [m
[31m-            num = this.joints.numjoints+1;[m
[31m-            [m
[31m-            this.joints.numjoints = num;[m
[31m-            this.joints.constrainedbody{num} = constrainedbodyname;[m
[31m-            this.joints.relativebody{num} = relativebodyname;[m
[31m-            this.joints.joint{num} = joint;[m
[31m-            this.joints.angleoffset(num) = angleoffset;[m
[31m-        [m
[31m-        end[m
[31m-        [m
[31m-        function [this] = addSpring(this,body1name,body2name,varargin)[m
[31m-            type = 'linear';[m
[31m-            springname = [];[m
[31m-            restlength = [];[m
[31m-            [m
[31m-            for i = 1 : 2 : length(varargin)[m
[31m-                option = varargin{i};[m
[31m-                val = varargin{i + 1};[m
[31m-                switch option[m
[31m-                    case 'type'[m
[31m-                        type = val;[m
[31m-                    case 'springname'[m
[31m-                        springname = val;[m
[31m-                    case 'restlength'[m
[31m-                        restlength = val;[m
[31m-                end[m
[31m-            end[m
[31m-            [m
[31m-            if ~strcmp(type,'linear') || ~strcmp(type,'angular')[m
[31m-                error('Unknown spring type.  Must pick linear or angular');[m
[31m-            end[m
[31m-            [m
[31m-            num = this.springs.numsprings+1;[m
[31m-            [m
[31m-            if ~isempty(springname)[m
[31m-                this.springs.name = springname;[m
[31m-            else[m
[31m-                this.springs.name = sym(sprintf('k%d',num));[m
[31m-            end[m
[31m-            [m
[31m-            if ~isempty(restlength)[m
[31m-                this.springs.restlength = restlength;[m
[31m-            else[m
[31m-                this.springs.restlength = sym(sprintf('springlength%d',num));[m
[31m-            end[m
[31m-            this.springs.numsprings = num;[m
[31m-            this.springs.body1{num} = body1name;[m
[31m-            this.springs.body2{num} = body2name;[m
[31m-            this.springs.type{num} = type;[m
[31m-        end[m
[31m-        [m
[31m-        function [this] = addDamper(this,body1name,body2name,varargin)[m
[31m-            type = 'linear';[m
[31m-            dampername = [];[m
[31m-            [m
[31m-            for i = 1 : 2 : length(varargin)[m
[31m-                option = varargin{i};[m
[31m-                val = varargin{i + 1};[m
[31m-                switch option[m
[31m-                    case 'type'[m
[31m-                        type = val;[m
[31m-                    case 'dampername'[m
[31m-                        dampername = val;[m
[31m-                end[m
[31m-            end[m
[31m-            [m
[31m-            if ~strcmp(type,'linear') || ~strcmp(type,'angular')[m
[31m-                error('Unknown damper type.  Must pick linear or angular');[m
[31m-            end[m
[31m-            [m
[31m-            num = this.dampers.numdampers+1;[m
[31m-            [m
[31m-            if ~isempty(dampername)[m
[31m-                this.dampers.name = dampername;[m
[31m-            else[m
[31m-                this.dampers.name = sym(sprintf('c%d',num));[m
[31m-            end[m
[31m-            [m
[31m-            this.dampers.numdampers = num;[m
[31m-            this.dampers.body1{num} = body1name;[m
[31m-            this.dampers.body2{num} = body2name;[m
[31m-            this.dampers.type{num} = type;[m
[31m-        end[m
[31m-        %% Symbolic Kinematics Calculations[m
[31m-        [m
[31m-        function [poscom] = PosCOM(this,body,varargin)[m
[31m-            RelTo = 'ground';[m
[31m-            [m
[31m-            for i = 1 : 2 : length(varargin)[m
[31m-                option = varargin{i};[m
[31m-                val = varargin{i + 1};[m
[31m-                switch option[m
[31m-                    case 'RelTo'[m
[31m-                        RelTo = val;[m
[31m-                end[m
[31m-            end[m
[31m-            [m
[31m-            poscom=[];[m
[31m-            [m
[31m-            [m
[31m-            [m
[31m-        end[m
[31m-        %% Symbolic Dynamics Calculations[m
[31m-        [m
[31m-        function [this] = getConstraintMatrix(this)[m
[31m-            %Creates a constraint matrix by interpreting the field[m
[31m-            %this.constraints[m
[31m-            [m
[31m-            [m
[31m-            j = this.joints;[m
[31m-            numjoints = j.numjoints;[m
[31m-            numbodies = this.numbodies;[m
[31m-            dof = this.dof;[m
[31m-            [m
[31m-            %Create symbols for state variables[m
[31m-            this.assignSymBodyStates;[m
[31m-            [m
[31m-            numconstraints = 0;[m
[31m-            for k = 1:numjoints[m
[31m-                [m
[31m-                b1name = j.relativebody{k};[m
[31m-                b2name = j.constrainedbody{k};[m
[31m-                [m
[31m-                %Assign state variables to body 1[m
[31m-                [b1,bnum1] = this.getBodyFromName(b1name);[m
[31m-                [px1,py1,pang1] = getSymBodyStates(this,b1name);[m
[31m-                %Assign symbolic variables to body 2[m
[31m-                [b2,bnum2] = this.getBodyFromName(b2name);[m
[31m-                [px2,py2,pang2] = getSymBodyStates(this,b2name);[m
[31m-                [m
[31m-                %Determine number of constraints imposed by joint k[m
[31m-                joint = j.joint{k};[m
[31m-                angleoffset = j.angleoffset(k);[m
[31m-                if strcmp(joint,'hinge') || strcmp(joint,'slider')