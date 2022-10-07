classdef Bar<Block
    properties
        diameter
        length
        mid_point
        rotation_matrix
    end

    methods
        function obj = Bar(varargin)
            %Vytvoří objekt tyče
            %Pořadí vstupů:
            %1. pozice, 
            %2. číslo bloku, 
            %3. adresa systému kam se přidává blok,
            %4. Průměr tyče, 
            %5. Délka tyče, 
            %6. Poloha středu tyče, 
            %7. Transformační matice
            obj = obj@Block(varargin{1:3}, "bar");
            obj.diameter = varargin{4};
            obj.length = varargin{5};
            obj.mid_point = varargin{6};
            obj.rotation_matrix = varargin{7};
        end

        function addBar(obj)
            if obj.DISP_SETTING
                disp("Přidávám block: "+obj.path())
            end
            add_block(obj.libPath(),...
                obj.path(), ...
                'Position',         obj.positionVector(),...
                'diameter',         num2str(obj.diameter),...
                'length',           num2str(obj.length), ...
                'initial_position', mat2str(obj.mid_point),...
                'initial_rotation', mat2str(obj.rotation_matrix))     
        end
    end
end