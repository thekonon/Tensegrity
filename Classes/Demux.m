classdef Demux<Block

    properties
        outputs
        input_name
    end


    methods
        function obj = Demux(varargin)
            %Vytvoří objekt muxu
            %pozice, číslo bloku, adresa systému, počet výstupů
            obj = obj@Block(varargin{1:3}, "demux");
            obj.outputs = varargin{4};
            obj.input_name = 'Cable_length';
        end

        function addDemux(obj)
            if obj.DISP_SETTING
                disp("Přidávám "+obj.name2path(obj.name, obj.number)+" na pozici ["+obj.position(1)+", "+obj.position(2)+"]")
            end
            add_block(obj.libPath(),...
                obj.path(), ...
                'Position', obj.positionVector(),...
                'Outputs', num2str(obj.outputs))
            add_block('simulink/Commonly Used Blocks/In1', [gcs, '/', obj.input_name], 'Position', obj.positionVector([-20 0]))
            add_line(gcs, [obj.input_name,'/1'], [obj.blockName,'/1'])

        end
    end
end