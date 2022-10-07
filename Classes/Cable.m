classdef Cable<Block
    properties
        specific_stiffness
        specific_dumping
        parallel_stiffness
        parallel_dumping
        free_length
    end

    methods
        function obj = Cable(varargin)
            %Vytvoří objekt lana
            %Pořadí vstupů:
            %pozice, číslo, ks, bs, kp, bp, l0
            %Kde ks,bs jsou měrné hodnoty lana
            %a kp, bp, l0 paralelní pružiny
            obj = obj@Block(varargin{1:3}, "cable");
            obj.specific_stiffness = varargin{4};
            obj.specific_dumping = varargin{5};
            obj.parallel_stiffness = varargin{6};
            obj.parallel_dumping = varargin{7};
            obj.free_length = varargin{8};
        end

        function addCable(obj)
            if obj.DISP_SETTING
                disp("Přidávám "+obj.path())
            end
            add_block(obj.libPath(),...
                obj.path(), ...
                'Position', obj.positionVector,...
                'k_s', num2str(obj.specific_stiffness),...
                'b_s', num2str(obj.specific_dumping),...
                'K_p', num2str(obj.parallel_stiffness),...
                'B_p', num2str(obj.parallel_dumping),...
                'l_0_p', num2str(obj.free_length)) 
        end
    end
end