classdef BaseModel
   
    properties
        
    end
    
    methods
        
        function [this,optimdata] = findLimitCycle(this,varargin)
            %Find a limit cycle for the model.
            
            %fmincon options
            plotiter = 1;
            TolCon = [];
            TolX = [];
            MaxEvals = [];
            Algorithm = 'interior-point';
            FinDiffType = [];
            
            for i = 1 : 2 : length(varargin)
                option = varargin{i};
                value = varargin{i + 1};
                switch option
                    case 'TolCon'
                        TolCon = value;
                    case 'Objective'
                        addedCostFcn = value;
                    case 'plotiter'
                        plotiter = value;
                    case 'TolX'
                        TolX = value;
                    case 'MaxEvals'
                        MaxEvals = value;
                    case 'Algorithm'
                        algorithm = value;
                    case 'FinDiffType'
                        FinDiffType= value;
                end
                
            end
            
            %Optimizer initial guess
            z0 = [this.x0(this.statestovary); this.ParmsFromList(this.parmstovary)];
            LB = -Inf*ones(size(z0));
            UB = Inf*ones(size(z0));
            for i = 1:length(this.parmstovary)
                LB(length(this.statestovary)+i) = 0;
                
            end
            
            
        end
        
    end
    
end