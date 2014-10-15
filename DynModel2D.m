classdef DynModel2D
    
    properties
        name = 'Model2D_1';
        bodies = [body2d.ground];
    end
    
    properties (Dependent = true)
        numbodies
    end
    
    
    
    methods
        
        function numbodies = get.numbodies(this)
           numbodies = length(this.bodies); 
        end
        
        function [this] = addBody(this,prevBody,varargin)
            %Creates a new body that has at most one DOF from the body it
            %connects to in the form of a hinge or a slider.  Must specify
            %the body it connects to.  It will be fixed to prevBody by
            %default.
            
            %'Hinge': set to 1 if you want a hinge joint
            %'Slider': give it a 2D unit vector for the direction that you
            %want it to slide.
            % NOTE: you can't give the body a hinge and a slider.  Make 2
            % separate bodies if you want to do this
            %'Mass','Inertia','d','lcom' are all parameters of the class
            %body2D that you can set as optional inputs here; otherwise,
            %they will be the default values
            
            
            hingeaxis = [];
            RelativeTo = [];
            slideraxis = [];
            Body = body2d;
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                val = varargin{i + 1};
                switch option
                    case 'Hinge'
                        hingeaxis = val;
                        Body.joint = 'hinge';
                    case 'RelativeTo'
                        RelativeTo = val;
                    case 'Slider'
                        slideraxis = val;
                        Body.joint = 'fixed';
                    case 'Mass'
                        Body.mass = val;
                    case 'Inertia'
                        Body.inertia = val;
                    case 'd'
                        Body.d = val;
                    case 'lcom'
                        Body.lcom = val;
                end
            end
            
            
            if ~isempty(hingeaxis) && ~isempty(slideraxis)
               error('You can''t define both a hinge and a slider for a new body') 
            end
            
            bodynumber = this.numbodies + 1;
            Body.bodynumber = bodynumber;

            % Define position states as q1,...,qn and velocity states as
            % u1,...,un, where n = number of bodies
            
            dofnumber = bodynumber-1;
            
            for i = 1:dofnumber
                name = sprintf('q%d',i);
                eval([sprintf('q%d',i) ' = sym(name);']);
                uname = sprintf('u%d',i);
                eval([sprintf('u%d',i) ' = sym(uname);']);
                
                qs(i)=eval(sprintf('q%d',i));
                us(i)=eval(sprintf('u%d',i));
            end
            
            prevpoint = prevBody.endpoint;
            
            if ~isempty(hingeaxis)
                 Body.joint = 'hinge';
                 
%                 if isempty(RelativeTo)
%                     angle = qs(bodynumber);
%                 else
%                     angle = qs(bodynumber) - this.angle(prevBodyNumber,1);
%                 end
                
                angle = qs(dofnumber);
                
                direction = [cos(angle);sin(angle)];
                
                Body.endpoint(1:2,bodynumber) = prevpoint + Body.d*direction;
                Body.compoint(1:2,bodynumber) = prevpoint + Body.lcom*direction;
                Body.angle(1,bodynumber) = angle;
                
            elseif ~isempty(slideraxis)
                
                Body.joint = 'slider';
                
                if Body.d ~= Body.lcom
                    Body.d = Body.lcom; %make endpoint of segment coincide with COM
                    warning('Setting Body.d to Body.lcom because Body is connected to prevBody by a Slider joint')
                end
                
                
                dist = qs(dofnumber);
                
                Body.endpoint(1:2,bodynumber,1:2) = prevpoint + dist*slideraxis;
                Body.compoint(1:2,bodynumber,1:2) = prevpoint + dist*slideraxis;
                Body.angle(1,bodynumber) = prevBody.angle;
                
            else %fixed joint
                Body.joint = 'fixed';
                
                angle = prevBody.angle;
                direction = [cos(angle);sin(angle)];
                
                Body.endpoint(1:2,bodynumber) = prevpoint + Body.d*direction;
                Body.compoint(1:2,bodynumber) = prevpoint + Body.lcom*direction;
                Body.angle(1,bodynumber) = angle;
            end
            
            this.bodies = [this.bodies Body];
            
        end
    end
    
end