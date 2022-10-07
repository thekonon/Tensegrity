classdef TopForce<Block
    properties
        force_vector
    end
    methods
        function obj = TopForce(varargin)
            %Vytvoří objekt síly do určitého uzlu - primárně zátěž konce
            %pozice, číslo bloku, adresa systému, vektor síly
            obj = obj@Block(varargin{1:3}, "top_force");
            obj.force_vector = varargin{4};
        end

        function addTopFrame(obj)
            if obj.DISP_SETTING
                disp("Přidávám "+obj.path())
            end
            add_block(obj.libPath(),...
                obj.path(), ...
                'Position', obj.positionVector,...
                'Fx', num2str(obj.force_vector(1)), ...
                'Fy', num2str(obj.force_vector(2)), ...
                'Fz', num2str(obj.force_vector(3)), ...
                'Mx', num2str(obj.force_vector(4)), ...
                'My', num2str(obj.force_vector(5)), ...
                'Mz', num2str(obj.force_vector(6)))
        end
    end
end