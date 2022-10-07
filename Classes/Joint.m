classdef Joint<Block
    properties
%         from_to
    end
    methods
        function obj = Joint(varargin)
            %Vytvoří objekt kloubu
            %1. Pozice bloku
            %2. Číslo bloku
            %3. Adresa systému
            obj = obj@Block(varargin{1:3}, "joint");
        end

        function addJoint(obj)
            if obj.DISP_SETTING
                disp("Přidávám "+obj.path())
            end
            add_block(obj.libPath(),...
                obj.path(), ...
                'Position', obj.positionVector)
        end
        
    end
end