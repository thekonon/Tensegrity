classdef Block<Constants
    properties
        position
        number
        type
        system_path
        size
        name
        library_name
        lib_name
        DISP_SETTING = 0
    end

    methods
        function obj = Block(varargin)
            obj.position = varargin{1};
            obj.number = varargin{2};
            obj.system_path = varargin{3};
            obj.type = varargin{4};
            
            obj.library_name = obj.my_library_name;
            switch obj.type
                case "bar"
                    obj.size = obj.bar_block_size;
                    obj.name = obj.bar_block_name;
                    obj.lib_name = obj.bar_name;
                case "cable"
                    obj.size = obj.cable_block_size;
                    obj.name = obj.cable_block_name;
                    obj.lib_name = obj.cable_name;
                case "bot_frame"
                    obj.size = obj.bot_frame_block_size;
                    obj.name = obj.bot_frame_block_name;
                    obj.lib_name = obj.bot_frame_name;
                case "joint"
                    obj.size = obj.joint_block_size;
                    obj.name = obj.joint_block_name;
                    obj.lib_name = obj.joint_name;
                case "top_frame"
                    obj.size = obj.top_frame_block_size;
                    obj.name = obj.top_frame_block_name;
                    obj.lib_name = obj.top_frame_name;
                case "mux"
                    obj.size = obj.mux_block_size;
                    obj.name = obj.mux_block_name;
                    obj.library_name = 'simulink/Commonly Used Blocks';
                    obj.lib_name = 'Mux';
                case "demux"
                    obj.size = obj.demux_block_size;
                    obj.name = obj.demux_block_name;
                    obj.library_name = 'simulink/Commonly Used Blocks';
                    obj.lib_name = 'Demux';
                case "scope"
                    obj.size = obj.scope_block_size;
                    if nargin == 5
                        obj.name = varargin{5};
                    else
                        obj.name = 'Scope';
                    end
                    obj.library_name = 'simulink/Commonly Used Blocks';
                    obj.lib_name = 'Scope';
                case "top_force"
                    obj.size = obj.top_force_block_size;
                    obj.name = obj.top_force_block_name;
                    obj.lib_name = obj.top_force_name;
                case "control"
                    obj.size = obj.control_block_size;
                    obj.name = obj.control_block_name;
                    obj.lib_name = obj.control_name;
                otherwise
                    obj.size = [70, 70];
                    obj.name = type;
            end
        end
        function string = blockName(obj)
            %Použití pro spojování bloků v add_line
            string = ([obj.name, num2str(obj.number)]);
        end
        function string = positionVector(obj, displacement, block_size)
            %Pozice bloku + posun -> vektor pozice pro Simulink
            %Použití: add_block pozice
            switch nargin
                case 1
                    string = mat2str([obj.position, obj.position+obj.size]);
                case 2
                    position_dummy = obj.position + displacement;
                    string = mat2str([position_dummy, position_dummy+obj.size]);
                case 3
                    position_dummy = obj.position + displacement;
                    string = mat2str([position_dummy, position_dummy+block_size]);
            end
        end
        function string = path(obj, i)
            %Zkrátí psaní   [gcs, '/', name]
            %Případně       [gcs, '/', name, i]
            %Použití: add_block cesta ke generovanému bloku
            switch nargin
                case 1
                    string = [obj.system_path, '/', obj.name, mat2str(obj.number)];
                case 2
                    string = [obj.system_path, '/', obj.name, mat2str(i)];
            end
        end
        function string = libPath(obj)
            %Použití: add_block adresa knihovny
            string = [obj.library_name,'/',obj.lib_name];
        end
    end
end