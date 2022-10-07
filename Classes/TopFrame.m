classdef TopFrame<Block
    properties
        r_base
        t_base
        nodes
        output_name = 'End efector'
        ps_port_name = 'End efector ps'
    end

    methods
        function obj = TopFrame(varargin)
            %Vytvoří objekt rámu
            %pozice, číslo bloku, adresa systému, poloměr báze, tloušťka
            %báze, posuny uzlů
            obj = obj@Block(varargin{1:3}, "top_frame");
            obj.r_base = varargin{4};
            obj.t_base = varargin{5};
            obj.nodes = varargin{6};
        end

        function addTopFrame(obj)
            if obj.DISP_SETTING
                disp("Přidávám "+obj.path())
            end
            add_block(obj.libPath(),...
                obj.path(), ...
                'Position', obj.positionVector,...
                'r_base', num2str(obj.r_base),...
                't_base', num2str(obj.t_base),...
                'node_1', mat2str(obj.nodes(:,1)),...
                'node_2', mat2str(obj.nodes(:,2)),...
                'node_3', mat2str(obj.nodes(:,3)))
            %Přidá výstup + spojí // jedno pro polohu end efektoru, jeden
            %výstup pro PS středu end efektoru
            add_block('simulink/Sinks/Out1', [obj.system_path,'/',obj.output_name],...
                'Position', obj.positionVector([20, -165] + obj.size, [30, 10]))
            add_line(gcs, [obj.blockName,'/1'], [obj.output_name,'/1'])
            add_block('nesl_utility/Connection Port', [obj.system_path, '/', obj.ps_port_name],...
                'Side', 'Right',...
                'Orientation', 'Left',...
                'Position', obj.positionVector([20, -65] + obj.size, [30, 10]))
            add_line(gcs, [obj.blockName,'/Rconn1'], [obj.ps_port_name,'/Rconn1'])
        end
    end
end