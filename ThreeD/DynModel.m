classdef DynModel
    
    properties
        name = 'Body1';
        mass = 1;
        inertia = 0;
        length = sym('l1');
        compos = 0.5;
        DOF = [0 0 1];
        axes = [1 1];
        endpoint = sym([0;0]);
        bodynumber = 0;
        
    end
    
    methods (Static)
       
        function [ground] = ground()
           ground = body2d;
           ground.mass = 0;
           ground.length = 0;
           ground.compos = 0;
           ground.DOF = [0 0 0];
        end
        
    end
    
    
    methods
        
        function [this] = body2d(this,prevBody,varargin)
            
            hingeaxis = [];
            RelativeTo = 'ground';
            slideraxis = [];
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'Hinge'
                        hingeaxis = val;
                    case 'RelativeTo'
                        RelativeTo = val;
                    case 'Slider'
                        slideraxis = val;
                        
                end
            end
            
            
            if ~isempty(hingeaxis) && ~isempty(slideraxis)
               error('You can''t define both a hinge and a slider for a new body') 
            end
            
            bodynumber = prevbody.bodynumber + 1;
            this.bodynumber = bodynumber;
            
            for i = 1:bodynumber
                name = sprintf('q%d',i);
                eval([sprintf('q%d',i) ' = sym(name);']);
                uname = sprintf('u%d',i);
                eval([sprintf('u%d',i) ' = sym(uname);']);
                
                qs(i)=eval(sprintf('q%d',i));
                us(i)=eval(sprintf('u%d',i));
            end
            
            if strcmp(prevBody,'ground')
                prevBody  = body2d.ground;
            end
            
            if ~isempty(hingeaxis)
                angle = qs(bodynumber);
                this.endpoint = prevbody.endpoint + this.length*[cos(angle);sin(angle)];
                
            end
            
        end
    end
    
end