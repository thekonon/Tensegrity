classdef Mux<Block

    properties
        inputs
        output_name
    end

    methods
        function obj = Mux(varargin)
            %Vytvoří objekt muxu
            %pozice, číslo bloku, adresa bloku, počet vstupů, jméno výstupního bloku
            obj = obj@Block(varargin{1:3}, "mux");
            obj.inputs = varargin{4};
            obj.output_name = varargin{5};
        end

        function addMux(obj)
            if obj.DISP_SETTING
                disp("Přidávám "+obj.path())
            end
            add_block(obj.libPath(),...
                obj.path(), ...
                'Position', obj.positionVector,...
                'Inputs', num2str(obj.inputs))
            add_block('simulink/Sinks/Out1', [gcs,'/',obj.output_name], 'Position', obj.positionVector([20 0]))
            add_line(gcs, [obj.blockName,'/1'], [obj.output_name,'/1'])
        end
    end
end