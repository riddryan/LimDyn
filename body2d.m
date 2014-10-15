classdef body2d
    
    properties
        bodynumber = 1; %give each body a number
        mass = sym('1'); 
        inertia = sym('0'); 
        d = sym('1'); %distance between previous body and the connection of the next body.  should be equal to lcom if it is a slider joint
        lcom = sym('.5'); %distance between previous body and center of mass
        joint='fixed'; %type of joint
         
        endpoint = sym([0;0]);
        compoint=sym([0 0]);
        angle = sym('0');
    end
    
    properties (Dependent = true)
        axes
    end
    
    methods (Static)
       
        function [ground] = ground()
           ground = body2d;
           ground.bodynumber = 1;
           ground.mass = sym('0');
           ground.d = sym('0');
           ground.lcom = sym('0');
           ground.endpoint = sym([0;0]);
           ground.compoint=sym([0 0]);
           ground.angle = sym('0');
           ground.joint = 'fixed';
        end
        
    end
    
    
    methods
        
        function axes = get.axes(this)
            axes = [cos(this.angle);sin(this.angle)];
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