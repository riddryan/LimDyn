classdef DynModel2D
    %A model defined by bodies, joints, and exteral forces which produce
    %symbolically defined dynamics equations and matrices.
    properties
        name = 'Model2D_1';
        bodies = body2d.empty;
%         joints = struct('constrainedbody',cell(1),'relativebody',cell(1),...
%             'joint',cell(1),'angleoffset',0,'numjoints',0); %joint info structure
        springs = struct('body1',cell(1),'body2',cell(1),'type',cell(1),'name',cell(1),'restlength',cell(1),'numsprings',0);
        dampers = struct('body1',cell(1),'body2',cell(1),'type',cell(1),'name',cell(1),'numdampers',0);
        phases = cell(1);
        groundslopeangle = sym(0); %Angle of the ground
        status = 0; % keeps track of whether new elements have been added to the model
        
        frames = struct; %Field for each DOF, subfields for each 2D unit vec in that frame
        positions = struct; %Field for each body, 2D vector for each body
        M = sym([]); %Maximal Mass Matrix
        C = {sym([])}; %Constraint Matrix
        Cdot = {sym([])}; %Derivative of constraint matrix
        G = sym([]); %Forces due to gravity
        SpringForces = sym([]); %Forces from Springs
        DamperForces = sym([]); %Forces from Dampers
        ExForces = sym([]); %External Forces (springs, dampers, etc.)
        RHS = sym([]); % Big right hand side, accounting for all Forces and Constraints
        MM = sym([]); %Big Mass Matrix, accounting for all masses and constraints
    end
    
    properties (Dependent = true)
        numbodies %number of rigid bodies
        gravity %gravity vector
        dof %degrees of freedom in the system
        posdexes %Indexes of position states
        veldexes %Indexes of velocity states
        bodydofs;
        mindofs;
        bodydofdexes;
        qs;
        us;
    end
    
    
    
    
    methods
        
       %%
       function [this] = Build(this)
           this.M = this.buildM;
           this.C = this.buildC;
           this.Cdot = this.buildCdot;
           this.G = this.buildG;
           this.SpringForces = this.buildSpringForces;
           this.DamperForces = this.buildDamperForces;
           this.ExForces = this.buildExForces;
           this.MM = this.buildMM;
           this.RHS = this.buildRHS;
           this.status = 0;
       end
        
        %% User-Input Model Specification
        
        function [this] = addBody(this,relativebodyname,joint,axis,varargin)
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
            
            this.status = this.status+1;
        end
        
        function [this] = addDOF(this,constrainedbodyname,relativebodyname,joint,varargin)
            %Stores the information of a new joint "joint" relative to the body2d
            %"relativebody" in this.joints
            
            %constrainedbodyname and relativebodyname are strings
            %corresponding to the constrained body and the body to which it
            %is constrained respectively
            
            %Possible arguments for JOINT
            % 'hinge' produces a hinge joint about the ground z-axis,
            % between the end of relativebody and constrainedbody
            % 'slider' produces a slider along an axis corresponding to the
            % angle of the relativebody
            %'fixed' constrains the constrainedbody to have no movement relative to
            %relativebody
            
            %axis only pertains to joints "fixed" and "slider".
            axis = [1 0 0]; %Axis for fixed and slider joints
            angleoffset = 0;
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                val = varargin{i + 1};
                switch option
                    case 'axis'
                        axis = val;
                    case 'angleoffset'
                        angleoffset = val;
                end
            end
            
            if strcmp(joint,'Hinge')
                axis = [0 0 1]; %2D, so rotation must be about z-axis
            end
            
            num = this.joints.numjoints + 1;
            this.joints.numjoints = num;
            this.joints.constrainedbody{num} = constrainedbodyname;
            this.joints.relativebody{num} = relativebodyname;
            this.joints.joint{num} = joint;
            this.joints.axis{num} = axis;
            this.joints.angleoffset(num) = angleoffset;
            this.status=this.status+1;
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
                this.springs.name{num} = springname;
            else
                this.springs.name{num} = sym(sprintf('k%d',num));
            end
            
            if ~isempty(restlength)
                this.springs.restlength{num} = restlength;
            else
                this.springs.restlength{num} = sym(sprintf('springlength%d',num));
            end
            this.springs.numsprings = num;
            this.springs.body1{num} = body1name;
            this.springs.body2{num} = body2name;
            this.springs.type{num} = type;
            this.status=this.status+1;
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
                this.dampers.name{num} = dampername;
            else
                this.dampers.name{num} = sym(sprintf('c%d',num));
            end
            
            this.dampers.numdampers = num;
            this.dampers.body1{num} = body1name;
            this.dampers.body2{num} = body2name;
            this.dampers.type{num} = type;
            this.status=this.status+1;
        end
        
        function [this] = addPhase(this,name,constraints,varargin)
            this.phases{end+1} = name;
            if ~isempty(constraints)
                this.C{end+1} = equationsToMatrix(constraints,r.us);
            else
                this.C{end+1} = [];
            end
            this.status = this.status+1;
        end
        
         %% Basic Access Methods
        function numbodies = get.numbodies(this)
            numbodies = length(this.bodies);
        end
        
        function dof = get.dof(this)
            dof = this.joints.numjoints;
        end
        
        function posdexes = get.posdexes(this)
            posdexes = 1:(this.dof/2);
        end
        
        function veldexes = get.veldexes(this)
            veldexes = (this.dof/2+1):this.dof;
        end
        
        function qs = get.qs(this)
           qs =  sym('q',[this.joints.numjoints 1]);
        end
        function us = get.us(this)
            us =  sym('u',[this.joints.numjoints 1]);
        end
        
        function this = set.bodies(this,newbodies)
            
            if ~isa(this.bodies,'body2d')
                error('this.bodies must be of class "body2d"')
            end
            
            names = cell(1,length(newbodies));
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
        
        function bodydofs = get.bodydofs(this)
            numbodies = this.numbodies;
            bodydofs = zeros(numbodies,1);
            for i = 1:numbodies
                bodydofs(i) = this.bodies(i).dof;
            end
        end
        
        function bodydofdexes = get.bodydofdexes(this)
            bodydofdexes = cumsum(this.bodydofs);
        end
        
        function mindofs = get.mindofs(this)
           mindofs = sum(this.bodydofs); 
        end
        
        %% Dynamics Calculations
        
        function positions = buildPositions(this)
            if ~this.status
                positions = this.positions;
                return;
            end
            
            for i = 1:this.numbodies
                
            end
        end
        
        function Cdot = buildCdot(this)
            
            if ~this.status
                Cdot = this.Cdot;
                return;
            end
            
            if isempty(this.C)
               Cdot = [];
               return;
            end
            
            C = this.C;
            
            for c = 1:length(C) %Each cell element of C corresponds to a phase
                [nconstraints,nstates] = size(C{c});
                Cdot{c} = sym(zeros(size(C{c})));
                for i = 1:nconstraints %each row/constraint
                    for j = 1:nstates %each column/DOF
                            for k = 1:length(this.qs) %each state
                                %Take the partial derivative of C(i,j) w.r.t.
                                %to each dof
                                Cdot(i,j) = eval(sprintf('Cdot{c}(i,j) + diff(C{c}(i,j),q%d)*u%d',k,k));
                            end
                    end
                end
            end
     
        end
        
        function G = buildG(this)
            if ~this.status
               G = this.G;
               return;
            end
            numbodies = this.numbodies;
            G = sym(zeros(numbodies*3,1));
            
            for i = 1:numbodies
                graveffect = this.bodies(i).mass*this.gravity;
                G(3*(i-1)+1,1) = graveffect(1);
                G(3*(i-1)+2,1) = graveffect(2);
                G(3*(i-1)+3,1) = 0;
            end

        end
        
        function M = buildM(this)
            if ~this.status
               M = this.M;
               return;
            end
            
            numbodies = this.numbodies;
            
            M = sym(zeros(numbodies*3));
            
            for i = 1:numbodies
                M(3*(i-1)+1,3*(i-1)+1) = this.bodies(i).mass;
                M(3*(i-1)+2,3*(i-1)+2) = this.bodies(i).mass;
                M(3*(i-1)+3,3*(i-1)+3) = this.bodies(i).inertia;
            end
            
        end
        
        function SpringForces = buildSpringForces(this)
            if ~this.status
                SpringForces = this.SpringForces;
                return;
            end
            numsprings = this.springs.numsprings;
            dof = this.dof;
            SpringForces = sym(zeros(dof,1));
            for i = 1:numsprings
                
                b1name = this.springs.body1{i};
                b2name = this.springs.body2{i};
                type = this.springs.type{i};
                
                
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
                    dex2x = 3*bnum2 - 2;
                    dex2y = 3*bnum2 - 1;
                    dex2ang = 3*bnum2;
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
                    newspringforces(dex2ang) = this.springs.name{i}*(ang1 - ang2 - this.springs.restlength{i});
                end
                SpringForces = SpringForces + newspringforces;
            end
        end
        
        function [DamperForces] = buildDamperForces(this)
            if ~this.status
                DamperForces = this.DamperForces;
                return;
            end
            numdampers = this.dampers.numdampers;
            dof = this.dof;
            DamperForces = sym(zeros(dof,1));
            %% Dampers
            for i = 1:numdampers
                b1name = this.dampers.body1{i};
                b2name = this.dampers.body2{i};
                type = this.dampers.type{i};
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
                    dex2x = 3*bnum2 - 2;
                    dex2y = 3*bnum2 - 1;
                    dex2ang = 3*bnum2;
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
                DamperForces = DamperForces + newdamperforces;
            end
        end
        
        function ExForces = buildExForces(this)
            if ~this.status
                ExForces = this.ExForces;
                return;
            end
            ExForces = this.SpringForces + this.DamperForces;
        end
        
        function RHS = buildRHS(this)
            if ~this.status
                RHS = this.RHS;
                return;
            end
            RHS = [this.G + this.ExForces; -this.Cdot*this.SymVels];
        end
        
        function MM = buildMM(this)
            if ~this.status
                MM = this.MM;
                return;
            end
            MM = [this.M this.C.'; this.C zeros(size(this.C,1))];
        end
        
        function J = buildJ(this)
%             if ~this.status
%                 J = this.J;
%                 return;
%             end
            
%            bodiesconstrainedtoground = this.joints.constrainedbody{strcmp(this.joints.relativebody,'ground')};
%            if isa(bodiesconstrainedtoground,'cell')
%                rootbody = bodiesconstrainedtoground{1};
%            else
%               rootbody = bodiesconstrainedtoground; 
%            end
%            
           J = sym(zeros(this.dof,this.mindofs));
           for i = 1:this.numbodies
               
           end
           
        end
        
        function BodyPositions = buildBodyPositions(this)
           this.assignMinStates;
           BodyPositions = sym(zeros(this.dof,1));
           
           prevbody = sym(zeros(3,1));
           prevangle = sym(0);
           for i = 1:this.numbodies
               jointtype = this.joints.joint{i};
               [qs,~] = this.getMinSymBodyStates(this.bodies(i).bodyname);
               if strcmp(jointtype,'Hinge')
                  BodyPositions(3*(i-1)+[1 2],1) = r.bodies(i).lcom*[cos(qs);sin(qs)]+prevbody(1:2);
                  BodyPositions(3*(i-1)+3,1) = qs + prevbody(3);
                  prevangle = qs;
               elseif strcmp(jointtype,'Slider')
                   axisangle = prevangle+this.joints.angleoffset{i};
                   BodyPositions(3*(i-1)+[1 2],1) = qs*[cos(axisangle);sin(axisangle)] + prevbody(1:2);
                   BodyPositions(3*(i-1)+3,1) = qs + prevbody(3);
               end
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
        
        function [qs,us] = getMinSymBodyStates(this,bodyname)
            [~,bodynum] = this.getBodyFromName(bodyname);
            num = this.bodydofs(bodynum);
            sdex = this.bodydofdexes(bodynum)-num+1;
            for i = 1:num
            qs(i,1) = sym(sprintf('q%d',sdex+i-1));
            us(i,1) = sym(sprintf('u%d',sdex+i-1)); 
            end
            
        end
        
        function [] = assignMinStates(this)
            ws = 'caller';
            %Create symbols for state variables
            for i = 1:this.mindofs
                assignin(ws, sprintf('q%d',i), eval(sprintf('sym(''q%d'');',i)) );
                assignin(ws, sprintf('u%d',i), eval(sprintf('sym(''u%d'');',i)) );
            end
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