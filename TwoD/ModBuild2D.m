classdef ModBuild2D
    %A model defined by bodies, joints, and exteral forces which produce
    %symbolically defined dynamics equations and matrices.
    properties
        name = 'Model2D_1';
        bodies = body2d.empty;
%         joints = struct('constrainedbody',cell(1),'relativebody',cell(1),...
%             'joint',cell(1),'angleoffset',0,'numjoints',0); %joint info structure
        springs = struct('body1',cell(1),'body2',cell(1),'type',cell(1),'name',cell(1),...
                         'restlength',cell(1),'attachvec1',cell(1),'attachvec2',cell(1),'numsprings',0);
        dampers = struct('body1',cell(1),'body2',cell(1),'type',cell(1),'name',cell(1),'numdampers',0);
        phases = cell(1); %A cell array of the names of the phases
        groundslopeangle = sym(0); %Angle of the ground
        status = 0; % keeps track of whether new elements have been added/changed in the model     
    end
    
    properties (SetAccess = protected)
        
        frames = struct; %Field for each DOF, subfields for each 2D unit vec in that frame
        Jv = sym([]); %Velocity jacobian
        Jw = sym([]); %Angular Velocity Jacobian
        J = sym([]); %Jacobian for all rigid body velocities
        M = sym([]); %Maximal Mass Matrix
        C = {sym([])}; %Constraint Matri
        Cdot = {sym([])}; %Derivative of constraint matrix
        G = sym([]); %Forces due to gravity
        CC = sym([]); %Coriolis & Centripetal Forces
        SpringForces = sym([]); %Forces from Springs
        DamperForces = sym([]); %Forces from Dampers
        ExForces = sym([]); %External Forces (springs, dampers, etc.)
        RHS = sym([]); % Big right hand side, accounting for all Forces and Constraints
        MM = sym([]); %Big Mass Matrix, accounting for all masses and constraints
        
    end
    
    properties (Dependent = true)
        bodynames
        numbodies %number of rigid bodies
        pos
        vel
        angvel
        gravity %gravity vector
        posdexes %Indexes of position states
        veldexes %Indexes of velocity states
        qs;
        us;
        allsyms;
    end
    
    
    
    
    methods
        
       %%
       function [this] = Build(this)
           this.Jv = this.buildJv;
           this.Jw = this.buildJw;
           this.J = this.buildJ;
           this.G = this.buildG;
           this.M = this.buildM;
           this.CC = this.buildCC;
           this.Cdot = this.buildCdot;
           
           this.SpringForces = this.buildSpringForces;
           this.DamperForces = this.buildDamperForces;
           this.ExForces = this.buildExForces;
           this.MM = this.buildMM;
           this.RHS = this.buildRHS;
           this.status = 0;
       end
        
        %% User-Input Model Specification
        
        function [this,Body] = addBody(this,bodyname,relativebody,joint,varargin)
            %Creates a new bodye.  You can set
            %its properties bodyname, mass, inertia, length (d), and lcom.
            %
            %relativebody is the body to which the joint is attached
            %
            % axis is a 2D unit vector that the slider joint moves along.
            %
            %Possible arguments for JOINT
            % 'hinge' produces a hinge joint about the ground z-axis,
            % between at the relativebody
            % 'slider' produces a slider along the axis specified
            %'fixed' constrains the body to have no movement relative to
            %relativebody
            
            mass = sym([]); inertia = sym([]); d = sym([0;0]); lcom = sym([]);
            angle = sym(0);
            
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
                    case 'angle'
                        angle = val;
                end
            end
            
            if strcmp(joint,'Hinge') && angle~=0
               warning('You cannot set Hinge angle in 2D.') 
            end
            
            Body = body2d;
            BodyPropNames = fieldnames(Body.bodyprops);
            Body.bodyname = bodyname;
            if this.numbodies~=0
                this.bodies = [this.bodies Body];
            else
                this.bodies = Body;
            end
           
            num = this.numbodies;
            
            %Assign dynamic properties of the new body
            for i = 1:length(BodyPropNames)
                %Assign the property to user input value
                Body.(BodyPropNames{i}) =  eval(BodyPropNames{i});
                
                %If user input no value, give it a default value based on
                %the number of the body that is
                if isempty(Body.(BodyPropNames{i}))
                        Body.(BodyPropNames{i}) = eval( sprintf('sym('' %s%d '')', BodyPropNames{i}, num) );
                end
                
            end
            
            
            %DOF/Joint information
            Body.relativebody = relativebody;
            Body.joint = joint;
            Body.q = sym(sprintf('q%d',num));
            Body.u = sym(sprintf('u%d',num));
            
            if strcmp(joint,'hinge')
                Body.angle = Body.q;
            else
                Body.angle = angle;
            end
            
            %Body position equal to prev body COM, plus the vector Body.d, plus
            %the length Body.lcom along the direction Body.angle
            if num>1
                Body.pos = this.bodies(num-1).pos + Body.d + Body.lcom*[cos(Body.angle);sin(Body.angle)];
            else
                Body.pos = Body.d + Body.lcom*[cos(Body.angle);sin(Body.angle)];
            end
            Body.pos = simplify(Body.pos);
            
            %Body velocity equal to time derivative of Body.pos
            for i = 1:this.numbodies
               Body.vel = Body.vel + diff(Body.pos,this.qs(i))*this.us(i);
               Body.angvel = Body.angvel + diff(Body.angle,this.qs(i))*this.us(i);
            end
            Body.vel = simplify(Body.vel); Body.angvel = simplify(Body.angvel);
            
            this.bodies(num) = Body;
            this.status = this.status+1;
        end
        
        function [this] = addSpring(this,body1name,body2name,varargin)
            type = 'linear';
            springname = []; %Name of Spring
            restlength = []; %Rest length of spring
            attachvec1 = sym([0;0]); %Where the spring is attached to body1 relative to its COM
            attachvec2 = sym([0;0]); %Where the spring is attached to body2 relative to its COM
            
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
                    case 'attachvec1'
                        attachvec1 = val;
                    case 'attachvec2'
                        attachvec2 = val;
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
            this.springs.attachvec1{num} = attachvec1;
            this.springs.attachvec2{num} = attachvec2;
            this.status=this.status+1;
        end
        
        function [this] = addDamper(this,body1name,body2name,varargin)
            type = 'linear';
            dampername = [];
                        attachvec1 = sym([0;0]); %Where the damper is attached to body1 relative to its COM
            attachvec2 = sym([0;0]); %Where the damper is attached to body2 relative to its COM
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                val = varargin{i + 1};
                switch option
                    case 'type'
                        type = val;
                    case 'dampername'
                        dampername = val;
                    case 'attachvec1'
                        attachvec1 = val;
                    case 'attachvec2'
                        attachvec2 = val;
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
            this.dampers.attachvec1{num} = attachvec1;
            this.dampers.attachvec2{num} = attachvec2;
            this.status=this.status+1;
        end
        
        function [this] = addPhase(this,name,constraints,varargin)
            this.phases{end+1} = name;
            if ~isempty(constraints)
                this.C{end+1} = equationsToMatrix(constraints,this.us);
            else
                this.C{end+1} = [];
            end
            this.status = this.status+1;
        end
        
         %% Access Methods
         function bodynames = get.bodynames(this)
             bodynames = cell(1,this.numbodies);
            for i = 1:this.numbodies
               bodynames{i} = this.bodies(i).bodyname; 
            end
         end
         function pos = get.pos(this)
             if this.numbodies==0
                pos = [];
                return;
             end
            for i = 1:this.numbodies
                pos.(this.bodies(i).bodyname) = this.bodies(i).pos;
            end
         end
         
         function vel = get.vel(this)
             if this.numbodies==0
                 vel = [];
                 return;
             end
             
             for i = 1:this.numbodies
                 vel.(this.bodies(i).bodyname) = this.bodies(i).vel;
             end
         end
         
         function angvel = get.angvel(this)
             if this.numbodies==0
                 angvel = [];
                 return;
             end
             
             for i = 1:this.numbodies
                 angvel.(this.bodies(i).bodyname) = this.bodies(i).angvel;
             end
         end
         
         function allsyms = get.allsyms(this)
             allsyms = {};
             for i = 1:this.numbodies
                     allsyms = [allsyms this.bodies(i).allsyms];
             end
             allsyms = unique(allsyms);
         end
         
        function numbodies = get.numbodies(this)
            numbodies = length(this.bodies);
        end
       
        
        function posdexes = get.posdexes(this)
            posdexes = 1:(this.dof/2);
        end
        
        function veldexes = get.veldexes(this)
            veldexes = (this.dof/2+1):this.dof;
        end
        
        function qs = get.qs(this)
           qs =  sym('q',[this.numbodies 1]);
        end
        function us = get.us(this)
            us =  sym('u',[this.numbodies 1]);
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
                error('You must assign a unique name to each body in the model')
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
        
        
        %% Dynamics Calculations
        
        function Jv = buildJv(this)
            if ~this.status
                Jv = this.Jv;
                return;
            end
            eqns = sym(zeros(2*this.numbodies,1));
           for i = 1:this.numbodies
              eqns(2*(i-1)+1) = this.vel.(this.bodynames{i})(1) == 0; 
              eqns(2*(i-1)+2) = this.vel.(this.bodynames{i})(2) == 0; 
           end
           Jv = simplify(equationsToMatrix(eqns,this.us));
        end
        function Jw = buildJw(this)
            if ~this.status
                Jw = this.Jw;
                return;
            end
            eqns = sym(zeros(this.numbodies,1));
            for i = 1:this.numbodies
                eqns(i) = this.angvel.(this.bodynames{i})(1) == 0;
            end
            Jw = equationsToMatrix(eqns,this.us);
            
            Jw = simplify(Jw);
        end
        
        function J = buildJ(this)
           if ~this.status
               J = this.J;
               return;
           end
           J = sym(zeros(3*this.numbodies, this.numbodies));
           for i = 1:this.numbodies
               J(3*(i-1)+(1:2),:) = this.Jv(2*(i-1)+(1:2),:);
               J(3*(i-1)+3,:) = this.Jw(i,:);
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
                                Cdot{c}(i,j) = Cdot{c}(i,j) + diff(C{c}(i,j),this.qs(k))*this.us(k);
                            end
                    end
                end
                Cdot{c} = simplify(Cdot{c});
            end
            
            
     
        end
        
        function G = buildG(this)
            if ~this.status
               G = this.G;
               return;
            end
            numbodies = this.numbodies;
            G = sym(zeros(numbodies*2,1));
            
            for i = 1:numbodies
                graveffect = this.bodies(i).mass*this.gravity;
                G(2*(i-1)+1,1) = graveffect(1);
                G(2*(i-1)+2,1) = graveffect(2);
            end
            
            G = simplify(transpose(this.Jv)*G);

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
            
            M = simplify(transpose(this.J) * M * this.J);
        end
        
        function CC = buildCC(this)
            
            if ~this.status
                CC = this.CC;
                return;
            end
            sz=size(this.M,1);
            c = sym(zeros(sz,sz,sz));
            CC = sym(zeros(sz,sz));
            for i=1:sz
                for j=1:sz
                    for k=1:sz
                        c(i,j,k)=1/2*(diff(this.M(i,j),this.qs(k)) + diff(this.M(i,k),this.qs(j)) - diff(this.M(k,j),this.qs(i)))*this.us(k);
                    end
                    CC(i,j) = sum(reshape(c(j,i,:),[sz 1]));
                end
            end
            CC=simplify(CC);
        end
        
        function SpringForces = buildSpringForces(this)
            if ~this.status
                SpringForces = this.SpringForces;
                return;
            end
            numsprings = this.springs.numsprings;
            dof = 3*this.numbodies;
            SpringForces = sym(zeros(dof,1));
            
            for i = 1:numsprings
                [b1,b1num] = this.getBodyFromName(this.springs.body1{i});
                [b2,b2num] = this.getBodyFromName(this.springs.body2{i});
                newspringforces = sym(zeros(dof,1));
                if strcmp(this.springs.type{i},'linear')
                    vec = b1.pos - b2.pos;
                    dir = vec/norm(vec);
                    dist = transpose(vec)*dir;
                    forcevec = -this.springs.name{i}*(dist - this.springs.restlength{i})*dir;
                    if b1num>0
                    newspringforces(3*(b1num-1)+(1:2),1) = forcevec;
                    end
                    if b2num>0
                    newspringforces(3*(b2num-1)+(1:2),1) = -forcevec;
                    end
                    
                    %Compute torque on body from attachvec offset
                        r1 = this.springs.attachvec1{i};
                        r2 = this.springs.attachvec2{i};
                        newspringforces(3*(b1num-1)+3,1) = r1(1)*forcevec(2) - r1(2)*forcevec(1);
                        newspringforces(3*(b2num-1)+3,1) = -(r2(1)*forcevec(2) - r2(2)*forcevec(1));
                    
                else
                    if b1num>0
                    newspringforces(3*(b1num-1)+3,1) = -this.springs.name{i}*(b1.angle - b2.angle - this.springs.restlength{i});
                    end
                    if b2num>0
                    newspringforces(3*(b2num-1)+3,1) = this.springs.name{i}*(b1.angle - b2.angle - this.springs.restlength{i});
                    end
                end
                SpringForces = SpringForces + newspringforces;
            end
            SpringForces = simplify(transpose(this.J) * SpringForces);
        end
        
        function [DamperForces] = buildDamperForces(this)
            if ~this.status
                DamperForces = this.DamperForces;
                return;
            end
            
            dof = 3*this.numbodies;
            DamperForces = sym(zeros(dof,1));
            
            for i = 1:this.dampers.numdampers
                [b1,b1num] = this.getBodyFromName(this.springs.body1{i});
                [b2,b2num] = this.getBodyFromName(this.springs.body2{i});
                newdamperforces = sym(zeros(dof,1));
                if strcmp(this.springs.type{i},'linear')
                    vec = b1.vel - b2.vel;
                    dir = vec/norm(vec);
                    dist = transpose(vec)*dir;
                    if b1num>0
                        newdamperforces(3*(b1num-1)+(1:2),1) = -this.dampers.name{i}*(dist)*dir;
                    end
                    if b2num>0
                        newdamperforces(3*(b2num-1)+(1:2),1) = this.dampers.name{i}*(dist)*dir;
                    end
                else
                    if b1num>0
                        newdamperforces(3*(b1num-1)+3,1) = -this.dampers.name{i}*(b1.angvel - b2.angvel);
                    end
                    if b2num>0
                        newdamperforces(3*(b2num-1)+3,1) = this.dampers.name{i}*(b1.angvel - b2.angvel);
                    end
                end
                DamperForces = DamperForces + newdamperforces;
            end
            DamperForces = simplify(transpose(this.J) * DamperForces);
        end
        
        function ExForces = buildExForces(this)
            if ~this.status
                ExForces = this.ExForces;
                return;
            end
            ExForces = simplify(this.SpringForces + this.DamperForces);
        end
        
        function RHS = buildRHS(this)
            if ~this.status
                RHS = this.RHS;
                return;
            end
            for i = 1:length(this.phases)
                if ~isempty(this.Cdot{i})
                RHS{i} = [this.G + this.ExForces; -this.Cdot{i}*this.us];
                else
                 RHS{i} = [this.G + this.ExForces];   
                end
                RHS{i} = simplify(RHS{i});
            end
        end
        
        function MM = buildMM(this)
            if ~this.status
                MM = this.MM;
                return;
            end
            for i = 1:length(this.phases)
                if ~isempty(this.C{i})
                MM{i} = [this.M this.C{i}.'; this.C zeros(size(this.C{i},1))];
                else
                   MM{i} = this.M; 
                end
                MM{i} = simplify(MM{i});
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
                bodynum = [];
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
        
        
        
        function [] = assignStates(this)
            ws = 'caller';
            %Create symbols for state variables
            for i = 1:this.numbodies
                assignin(ws, sprintf('q%d',i), eval(sprintf('sym(''q%d'');',i)) );
                assignin(ws, sprintf('u%d',i), eval(sprintf('sym(''u%d'');',i)) );
            end
        end
        
        function export(this)
            s = mkdir(this.name);
            classdir = [cd '\' this.name '\'];
            fid = fopen([classdir this.name],'w');
            
%             fprintf(fid,
            
            fclose(fid);
        end
        
        
        
        
    end
    
end