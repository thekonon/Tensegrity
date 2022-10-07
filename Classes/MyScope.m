classdef MyScope<Block

    properties
        sample_time = 0
    end

    methods
        function obj = MyScope(varargin)
            %Vytvoří objekt muxu
            %pozice, číslo bloku, adresa systému, jméno scopu
            obj = obj@Block(varargin{1:3}, "scope", varargin{4});
            if nargin == 5
                obj.sample_time = varargin{5};
            end
        end

        function addScope(obj)
            if obj.DISP_SETTING
                disp("Přidávám "+obj.path())
            end
            if obj.sample_time == 0
                add_block(obj.libPath(),...
                    obj.path(), ...
                    'Position', obj.positionVector)
            else
                add_block(obj.libPath(),...
                    obj.path(), ...
                    'Position', obj.positionVector,...
                    'SampleTime', num2str(obj.sample_time))
            end
        end
    end
end