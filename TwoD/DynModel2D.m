classdef DynModel2D
    %A model defined by bodies, joints, and exteral forces which produce
    %symbolically defined dynamics equations and matrices.
    properties
        name = 'Model2D_1';
        bodies = body2d.empty;
        joints = struct('constrainedbody',cell(1),'relativebody',cell(1),...
            'joint',cell(1),'angleoffset',0,'numjoints',0); %joint info structure
        springs = struct('body1',cell(1),'body2',cell(1),'type',cell(1),'name',cell(1),'restlength',cell(1),'numsprings',0);
        dampers = struct('body1',cell(1),'body2',cell(1),'type',cell(1),'name',cell(1),'numdampers',0);
        groundslopeangle = sym(0);
        M = sym([]); %Maximal Mass Matrix
        C = sym([]); %Constraint Matrix
        Cdot = sym([]); %Derivative of constraint matrix
        G = sym([]); %Forces due to gravity
        ExForces = sym([]); %External Forces (springs, dampers, etc.)
    end
    
    properties (Dependent = true)
        numbodies %number of rigid bodies
        gravity
        dof
        posdexes
        veldexes
    end
    
    
    
    methods
        
        %% Access Methods
        function numbodies = get.numbodies(this)
            numbodies = length(this.bodies);
        end
        
        function dof = get.dof(this)
            dof = 3*this.numbodies;
        end
        
        function posdexes = get.posdexes(this)
            posdexes = 1:(this.dof/2);
        end
        
        function veldexes = get.veldexes(this)
            veldexes = (this.dof/2+1):this.dof;
        end
        
        function this = set.bodies(this,newbodies)
            
            if ~isa(this.bodies,'body2d')
                error('this.bodies must be of class "body2d"')
            end
            
            for i = 1:length(newbodies)
                names{i} = newbodies(i).bodyname;
            end
            uniquenames = unique(names);
            
            if length(uniquenames)<length(newbodies)
                error('You must assign a new name to each body in the model')
            else
                this.bodies = newbodies;
            end
        end
        
        function gravity = get.gravity(this)
            gravity = sym([0;-1]);
            alpha = this.groundslopeangle;
            Rotation = [cos(alpha) sin(alpha); -sin(alpha) cos(alpha)];
            gravity = Rotation*gravity;
        end
        
        %% User-Input Model Specification
        
        function [this] = addBody(this,varargin)
            %Creates a new bodye.  You can set
            %its properties bodyname, mass, inertia, length (d), and lcom.
            mass = sym([]); inertia = sym([]); d = sym([]); lcom = sym([]); bodyname = char([]);
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                val = varargin{i + 1};
                switch option
                    case 'mass'
                        mass = val;
                    case 'inertia'
                        inertia = val;
                    case 'd'
                        d = val;
                    case 'lcom'
                        lcom = val;
                    case 'bodyname'
                        bodyname = val;
                end
            end
            
            Body = body2d;
            BodyPropNames = fieldnames(Body.bodyprops);
            oldnumberofbodies = this.numbodies;
            
            %Assign dynamic properties of the new body
            for i = 1:length(BodyPropNames)
                %Assign the property to user input value
                Body.(BodyPropNames{i}) =  eval(BodyPropNames{i});
                
                %If user input no value, give it a default value based on
                %the number of the body that is
                if isempty(Body.(BodyPropNames{i}))
                    Body.(BodyPropNames{i}) = eval( sprintf('sym('' %s%d '')', BodyPropNames{i}, oldnumberofbodies+1) );
                end
                
            end
            
            
            %If user input no value, give it name BodyX, where X is the
            %number of the new body
            if isempty(bodyname)
                Body.bodyname = eval( sprintf('Body%d', oldnumberofbodies+1) );
            else %Assign body name based on user input
                Body.bodyname = bodyname;
            end
            
            if oldnumberofbodies~=0
                this.bodies = [this.bodies Body];
            else
                this.bodies = Body;
            end
            
        end
        
        function [this] = addJoint(this,constrainedbodyname,relativebodyname,joint,varargin)
            %Stores the information of a new joint "joint" relative to the body2d
            %"relativebody" in this.joints
            
            %angleoffset only pertains to joints "fixed" and "slider".  The
            %value of angle offset determines the CCW rotational offset of
            %constrainedbody relative to relativebody.
            
            angleoffset = 0; %only for fixed and slider joints
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                val = varargin{i + 1};
                switch option
                    case 'angleoffset'
                        angleoffset = val;
                end
            end
            
            if strcmp(joint,'Hinge')
                angleoffset = 0;
            end
            
            num = this.joints.numjoints+1;
            
            this.joints.numjoints = num;
            this.joints.constrainedbody{num} = constrainedbodyname;
            this.joints.relativebody{num} = relativebodyname;
            this.joints.joint{num} = joint;
            this.joints.angleoffset(num) = angleoffset;
            
        end
        
        function [this] = addSpring(this,body1name,body2name,varargin)
            type = 'linear';
            springname = [];
            restlength = [];
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                val = varargin{i + 1};
                switch option
                    case 'type'
                        type = val;
                    case 'springname'
                        springname = val;
                    case 'restlength'
                        restlength = val;
                end
            end
            
            if ~strcmp(type,'linear') && ~strcmp(type,'angular')
                error('Unknown spring type.  Must pick linear or angular');
            end
            
            num = this.springs.numsprings+1;
            
            if ~isempty(springname)
                this.springs.name = springname;
            else
                this.springs.name = sym(sprintf('k%d',num));
            end
            
            if ~isempty(restlength)
                this.springs.restlength = restlength;
            else
                this.springs.restlength = sym(sprintf('springlength%d',num));
            end
            this.springs.numsprings = num;
            this.springs.body1{num} = body1name;
            this.springs.body2{num} = body2name;
            this.springs.type{num} = type;
        end
        
        function [this] = addDamper(this,body1name,body2name,varargin)
            type = 'linear';
            dampername = [];
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                val = varargin{i + 1};
                switch option
                    case 'type'
                        type = val;
                    case 'dampername'
                        dampername = val;
                end
            end
            
            if ~strcmp(type,'linear') && ~strcmp(type,'angular')
                error('Unknown damper type.  Must pick linear or angular');
            end
            
            num = this.dampers.numdampers+1;
            
            if ~isempty(dampername)
                this.dampers.name = dampername;
            else
                this.dampers.name = sym(sprintf('c%d',num));
            end
            
            this.dampers.numdampers = num;
            this.dampers.body1{num} = body1name;
            this.dampers.body2{num} = body2name;
            this.dampers.type{num} = type;
        end
        %% Symbolic Kinematics Calculations
        
        function [poscom] = PosCOM(this,body,varargin)
            RelTo = 'ground';
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                val = varargin{i + 1};
                switch option
                    case 'RelTo'
                        RelTo = val;
                end
            end
            
            poscom=[];
            
            
            
        end
        %% Symbolic Dynamics Calculations
        
        function [this] = getConstraintMatrix(this)
            %Creates a constraint matrix by interpreting the field
            %this.constraints
            
            
            j = this.joints;
            numjoints = j.numjoints;
            dof = this.dof;
            
            %Create symbols for state variables
            this.assignSymBodyStates;
            vels = this.SymVels;
            
            numconstraints = 0;
            for k = 1:numjoints
                
                b1name = j.relativebody{k};
                b2name = j.constrainedbody{k};
                
                %Assign state variables to bodies
                [b1,bnum1] = this.getBodyFromName(b1name);
                [x1,y1,ang1,vx1,vy1,vang1] = getSymBodyStates(this,b1name);
                [b2,bnum2] = this.getBodyFromName(b2name);
                [x2,y2,ang2,vx2,vy2,vang2] = getSymBodyStates(this,b2name);
                
                %Determine number of constraints imposed by joint k
                joint = j.joint{k};
                angleoffset = j.angleoffset(k);
                if strcmp(joint,'hinge') || strcmp(joint,'slider')
                    numconstraints = numconstraints + 2;
                elseif strcmp(joint,'fixed')
                    numconstraints = numconstraints + 3;
                end
                
                newconstraints = sym(zeros(numconstraints,1));
                if strcmp(joint,'slider') || strcmp(joint,'fixed')
                    % If not a hinge, set angular velocity of b2 to angular
                    % velocity of b1
                    newconstraints(1) = vang1 - vang2 == 0;
                    %Angle of constrainedbody
                    alpha = ang1 + angleoffset;
                    %Do not allow velocity perpendicular to the direction of
                    %the body
                    newconstraints(2) = vx1*-sin(alpha) + vx2*cos(alpha) == 0;
                end
                
                %Do not allow velocity along the direction of the body
                if strcmp(joint,'fixed')
                    newconstraints(2) = vx1*cos(alpha) + vx2*sin(alpha) == 0;
                end
                
                if strcmp(joint,'hinge')
                    newconstraints(1) = vx1 - (b1.d - b1.lcom)*sin(ang1)*vang1 - vx2 - b2.lcom*sin(ang2)*vang2 == 0;
                    newconstraints(2) = vy1 + (b1.d - b1.lcom)*cos(ang1)*vang1 - vy2 + b2.lcom*cos(ang2)*vang2 == 0;
                end
                newconstraints = equationsToMatrix(newconstraints,vels);
                %append constraints
                this.C(numconstraints - size(newconstraints,1) + 1 : numconstraints,:) = newconstraints;
                
            end
            
        end
        
        function [this] = getConstraintMatrixDot(this)
            if isempty(this.C)
                this = this.getConstraintMatrix;
            end
            C = this.C;
            
            [nconstraints,nstates] = size(C);
            numbodies = this.numbodies;
            
            Cdot = sym(zeros(size(C)));
            this.assignSymBodyStates;
            types = [{'x'} {'y'} {'ang'}];
            
            for i = 1:nconstraints %each row-element of Cdot
                for j = 1:nstates %each column-element of Cdot
                    for k = 1:numbodies %each body in the model
                        for t = 1:length(types) %each state of the body (x,y,ang)
                            %Take the partial derivative of C(i,j) w.r.t.
                            %to state types(t) of body k
                            Cdot(i,j) = eval(sprintf('Cdot(i,j) + diff(C(i,j),%s%d)*v%s%d',types{t},k,types{t},k));
                        end
                    end
                end
            end
            
            this.Cdot = Cdot;
            
            
        end
        
        function [this] = getGravityForces(this)
            numbodies = this.numbodies;
            G = sym(zeros(numbodies*3,1));
            
            for i = 1:numbodies
                graveffect = this.bodies(i).mass*this.gravity;
                G(3*(i-1)+1,1) = graveffect(1);
                G(3*(i-1)+2,1) = graveffect(2);
                G(3*(i-1)+3,1) = 0;
            end
            
            this.G = G;
        end
        
        function [this] = getMassMatrix(this)
            
            numbodies = this.numbodies;
            
            M = sym(zeros(numbodies*3));
            
            for i = 1:numbodies
                M(3*(i-1)+1,3*(i-1)+1) = this.bodies(i).mass;
                M(3*(i-1)+2,3*(i-1)+2) = this.bodies(i).mass;
                M(3*(i-1)+3,3*(i-1)+3) = this.bodies(i).inertia;
            end
            
            this.M = M;
        end
        
        function [this] = getExForces(this)
            numsprings = this.springs.numsprings;
            numdampers = this.dampers.numdampers;
            dof = this.dof;
            %% Springs
            for i = 1:numsprings
                
                b1name = j.relativebody{i};
                b2name = j.constrainedbody{i};
                
                %Assign state variables to bodies
                [b1,bnum1] = this.getBodyFromName(b1name);
                if ~strcmp(b1name,'ground')
                    [x1,y1,ang1,vx1,vy1,vang1] = getSymBodyStates(this,b1name);
                    dex1x = 3*bnum1 - 2;
                    dex1y = 3*bnum1 - 1;
                    dex1ang = 3*bnum1;
                else
                    x1=[];y1=[];ang1=[];vx1=[];vy1=[];vang1=[];
                    dex1x = []; dex1y = []; dex1ang = [];
                end
                
                [b2,bnum2] = this.getBodyFromName(b2name);
                if ~strcmp(b2name,'ground')
                    [x2,y2,ang2,vx2,vy2,vang2] = getSymBodyStates(this,b2name);
                    dex1x = 3*bnum2 - 2;
                    dex1y = 3*bnum2 - 1;
                    dex1ang = 3*bnum2;
                else
                    x2=[];y2=[];ang2=[];vx2=[];vy2=[];vang2=[];
                    dex2x = []; dex2y = []; dex2ang = [];
                end
                
                newspringforces = sym(zeros(dof,1));
                if strcmp(type,'linear')
                    angle = atan2((y2-y1),(x2-x1));
                    newspringforces([dex1x dex1y]) =  -this.springs.name{i}*[(x1 - x2 - this.springs.restlength{i}*cos(angle)) (y1 - y2 - this.springs.restlength{i}*sin(angle))];
                    newspringforces([dex2x dex2y]) =  this.springs.name{i}*[(x1 - x2 - this.springs.restlength{i}*cos(angle)) (y1 - y2 - this.springs.restlength{i}*sin(angle))];
                else %angular
                    newspringforces(dex1ang) = -this.springs.name{i}*(ang1 - ang2 - this.springs.restlength{i});
                    newspringforces(dex2ang) = this.springs.name{i}*(ang2 - ang2 - this.springs.restlength{i});
                end
                this.ExForces = this.ExForces + newspringforces;
            end
            %% Dampers
            for i = 1:numdampers
                b1name = this.dampers.body1{i};
                b2name = this.dampers.body2{i};
                
                %Assign state variables to bodies
                [b1,bnum1] = this.getBodyFromName(b1name);
                if ~strcmp(b1name,'ground')
                    [x1,y1,ang1,vx1,vy1,vang1] = getSymBodyStates(this,b1name);
                    dex1x = 3*bnum1 - 2;
                    dex1y = 3*bnum1 - 1;
                    dex1ang = 3*bnum1;
                else
                    x1=[];y1=[];ang1=[];vx1=[];vy1=[];vang1=[];
                    dex1x = []; dex1y = []; dex1ang = [];
                end
                
                [b2,bnum2] = this.getBodyFromName(b2name);
                if ~strcmp(b2name,'ground')
                    [x2,y2,ang2,vx2,vy2,vang2] = getSymBodyStates(this,b2name);
                    dex1x = 3*bnum2 - 2;
                    dex1y = 3*bnum2 - 1;
                    dex1ang = 3*bnum2;
                else
                    x2=[];y2=[];ang2=[];vx2=[];vy2=[];vang2=[];
                    dex2x = []; dex2y = []; dex2ang = [];
                end
                
                newdamperforces = sym(zeros(dof,1));
                if strcmp(type,'linear')
                        newdamperforces([dex1x dex1y]) = -this.dampers.name{i}*[(vx1-vx2) (vy1-vy2)];
                        newdamperforces([dex2x dex2y]) =  this.dampers.name{i}*[(vx1-vx2) (vy1-vy2)];
                else
                        newdamperforces(dex1ang) = -this.dampers.name{i}*(vang1 - vang2);
                        newdamperforces(dex2ang) = this.dampers.name{i}*(vang1 - vang2);
                end
                this.ExForces = this.ExForces + newdamperforces;
            end
        end
        
        %% Simulation
        function [xddot,constraintforces] = XDoubleDot(this,time,state)
            RHS = [this.G + this.ExForces; -this.Cdot*state(this.veldexes)];
            bigM = [this.M this.C.'; this.C zeros(size(this.C.'))];
            AccsAndConstraints = bigM \ RHS;
            
            accs = AccsAndConstraints(1:this.dof/2);
            xddot = [state(this.posdexes);accs];
            constraintforces = AccsAndConstraints(this.dof/2+1:end);
        end
        
        %% Events
        
        %% Housekeeping
        
        function [body,bodynum] = getBodyFromName(this,bodyname)
            
            if strcmp('ground',bodyname)
                body = body2d.ground;
                bodynum = 0;
                return;
            end
            for i = 1:this.numbodies
                if strcmp(this.bodies(i).bodyname,bodyname)
                    body = this.bodies(i);
                    bodynum = i;
                    break;
                end
            end
        end
        
        function [x,y,ang,vx,vy,vang] = getSymBodyStates(this,bodyname)
            
            if strcmp('ground',bodyname)
                x = sym(0); vx = sym(0);
                y = sym(0); vy = sym(0);
                ang = sym(0); vang = sym(0);
                return;
            end
            
            [~,bodynum] = this.getBodyFromName(bodyname);
            x = sym(sprintf('x%d',bodynum));
            y = sym(sprintf('y%d',bodynum));
            ang = sym(sprintf('ang%d',bodynum));
            vx = sym(sprintf('vx%d',bodynum));
            vy = sym(sprintf('vy%d',bodynum));
            vang = sym(sprintf('vang%d',bodynum));
        end
        
        function [] = assignSymBodyStates(this)
            numbodies = this.numbodies;
            ws = 'caller';
            %Create symbols for state variables
            for i = 1:numbodies
                assignin(ws, sprintf('x%d',i), eval(sprintf('sym(''x%d'');',i)) );
                assignin(ws, sprintf('y%d',i), eval(sprintf('sym(''y%d'');',i)) );
                assignin(ws, sprintf('ang%d',i), eval(sprintf('sym(''ang%d'');',i)) );
                assignin(ws, sprintf('vx%d',i), eval(sprintf('sym(''vx%d'');',i)) );
                assignin(ws, sprintf('vy%d',i), eval(sprintf('sym(''vy%d'');',i)) );
                assignin(ws, sprintf('vang%d',i), eval(sprintf('sym(''vang%d'');',i)) );
            end
        end
        
        function vels = SymVels(this)
            numbodies = this.numbodies;
            vxs = sym('vx',[1 numbodies]);
            vys = sym('vy',[1 numbodies]);
            vangs = sym('vang',[1 numbodies]);
            vels = [vxs; vys; vangs];
            vels = reshape(vels,numbodies*3,1);
        end
        
        function BodyNames = getBodyNames(this)
            BodyNames = cell(1,this.numbodies);
            for i = 1:this.numbodies;
                BodyNames{1,i} = this.bodies(i).bodyname;
            end
        end
        
        
        
    end
    
end