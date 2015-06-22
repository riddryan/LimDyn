classdef body2d
    %A 2-d rigid body, to be used in an instance of DynModel2D.
    
    
    properties
        bodyname = 'Body0';
        mass = sym('1'); 
        inertia = sym('0'); 
        d = sym([0;0]); %distance from relative body's center of mass to the joint
        lcom = sym('1'); %distance between start of body to COM of body
        op = struct; %Other parameters
        
        q = sym([]);
        u = sym([]);
        angle = sym([]); %angle of x axis from ground
        pos = sym([]);
        vel = sym([]);
        
        %DOF info
        relativebody = []; %Body to which joint is attached
        joint = []; %type of joint
        jointaxis = [ ]; %axis of joint
    end
    
    properties (Dependent = true)
        bodyprops
        allsyms;
        R; %Rotation Matrix from ground frame to body Frame
        xaxis; %xaxis of body frame
        yaxis; %yaxis of body frame
    end
    
    methods (Static)
       
        function [ground] = ground()
           ground = body2d;
           ground.bodyname = 'ground';
           ground.mass = sym('0');
           ground.d = sym([0 0]);
           ground.lcom = sym('0');
        end
        
    end
    
    
    methods
        
        function R = get.R(this)
           R = [cos(this.ang) sin(this.ang); -sin(this.ang) cos(this.ang)]; 
        end
        
        function xaxis = get.xaxis(this)
            xaxis = this.R*[1;0];
        end
        
        function yaxis = get.yaxis(this)
            yaxis = this.R*[0;1];
        end
        
        function allsyms = get.allsyms(this)
            allsyms = {};
            fnames = fieldnames(this.bodyprops);
            for i = 1:length(fnames)
                for j = 1:length(this.(fnames{i}))
                    allsyms = [allsyms symvar(this.(fnames{i})(j))];
                end
            end
            allsyms = unique(allsyms);
        end
        
        function bodyprops = get.bodyprops(this)
            bodyprops.mass = this.mass;
            bodyprops.inertia = this.inertia;
            bodyprops.d = this.d;
            bodyprops.lcom = this.lcom;
        end
        
        function [] = getParams(this)
            %%
            % Add the parameter variables dynamically to the work space of the
            % function that calls this function.
            ws = 'caller';
            
            parmnames = fieldnames(this);
            for i = 1:length(parmnames)
                pvalue = this.(parmnames{i});
                if ~isa(pvalue,'sym')
                    pstring = sprintf(['assignin(ws,''' parmnames{i} ''', %g);'],pvalue);
                else
                    pstring = ['assignin(ws,''' parmnames{i} ''', pvalue);'];
                end
                eval(pstring);
            end
            
        end
        
        function [this] = setParametersFromList(this, parameterNames, parameterValues)
            %%
            for i = 1 : length(parameterNames)
                this.(parameterNames{i}) = parameterValues(i);
            end
        end
        

    end
    
end