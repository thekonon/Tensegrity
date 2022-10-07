classdef BotFrame<Block
    properties
        r_base
        t_base
        nodes
    end

    methods
        function obj = BotFrame(varargin)
            %Vytvoří objekt rámu
            %pozice, číslo bloku, adresa systému, poloměr báze, tloušťka báze
            obj = obj@Block(varargin{1:3}, "bot_frame");
            obj.r_base = varargin{4};
            obj.t_base = varargin{5};
            obj.nodes = varargin{6};
        end

        function addBotFrame(obj)
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
        end
    end
end