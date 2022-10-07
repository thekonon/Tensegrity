classdef (Abstract) Constants
    %Obsahuje jen jména pro generování

    properties(Constant, Access = public)
%         %Názvy
%         system_name = 'Vygenerovana_tensegrita'
%         subsystem_name = 'Robot'

        %Názvy bloků v knihovně
        my_library_name = 'TensegrityLibrary'
        bot_frame_name = 'BotFrame'
    end
    properties(Constant, Access = protected)
        %Jména bloků v knihovně
        bar_name = 'Bar'
        cable_name = 'Cable'
        joint_name = 'Joint'
        top_frame_name = 'TopFrame'
        top_force_name = 'TopForce'
        control_name = 'TensegrityControl'

        %Názvy bloků ve vygenerovaném modelu
        bot_frame_block_name = 'Bot frame'
        bar_block_name = 'Bar'
        cable_block_name = 'Cable'
        joint_block_name = 'Joint'
        top_frame_block_name = 'Top frame'
        top_force_block_name = 'Top force'
        mux_block_name = 'My mux'
        demux_block_name = 'My demux'
        control_block_name = 'Control'

        %Nastavení konstantních parametrů vzhledu vygenerovaného modelu
        subsystem_block_size =  [200 100]
        bot_frame_block_size = ceil([982 640]/3)
        bar_block_size =        [200 100]
        cable_block_size =      [200 100]
        joint_block_size =      [75 75]
        mux_block_size =        [10 80]
        demux_block_size =      [10 80]
        top_frame_block_size =  ceil([982 640]/3)
        top_force_block_size =  [200 100]
        control_block_size =    [80 80]
        scope_block_size =      [70 70]
    end
end