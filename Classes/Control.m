classdef Control<Block

    properties
        free_length
    end

    methods
        function obj = Control(varargin)
            %Vytvoří objekt muxu
            %pozice, číslo bloku, adresa systému, konstanta
            obj = obj@Block(varargin{1:3}, "control");
            obj.free_length = varargin{4};
        end

        function addControl(obj)
            if obj.DISP_SETTING
                disp("Přidávám "+obj.path())
            end
            add_block(obj.libPath(),...
                obj.path(), ...
                'Position', obj.positionVector)
            obj.setFreeLength();
        end
        
        function obj = setFreeLength(obj, free_length)
            switch nargin
                case 1
                    set_param(obj.path, 'free_length', mat2str(obj.free_length))
                case 2
                    set_param(obj.path, 'free_length', mat2str(free_length))
                    obj.free_length = free_length;
            end
        end
    end
end