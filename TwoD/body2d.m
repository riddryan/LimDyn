classdef body2d
    %A 2-d rigid body, to be used in an instance of DynModel2D.
    
    
    properties
        bodyname = 'Body0';
        mass = sym('1'); 
        inertia = sym('0'); 
        d = sym('1'); %distance from start of body to end of body
        lcom = sym('.5'); %distance between start of body to COM of body
        q = sym([]);
        u = sym([]);
        xaxis = sym([]);
        yaxis = sym([]);
        angle = sym([]);
        
        %DOF info
        relativebody = [];
        joint = [];
        axis = [ ];
    end
    
    properties (Dependent = true)
        bodyprops
    end
    
    methods (Static)
       
        function [ground] = ground()
           ground = body2d;
           ground.bodyname = 'ground';
           ground.mass = sym('0');
           ground.d = sym('0');
           ground.lcom = sym('0');
        end
        
    end
    
    
    methods
        
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