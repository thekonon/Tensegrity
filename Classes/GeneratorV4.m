classdef GeneratorV4 < TensegritySettings
    %Vytvoří instanci objektu generátoru používaného pro generování
    %a řízení tensegrit
    properties(Access = private)
        end_cables_position
        end_bars_position

    end
    properties(Access = public)
        bars_list       Bar
        cables_list     Cable
        joint_list      Joint
        mux_list        Mux
        demux_list      Demux
        bot_frame_list  BotFrame
        top_frame_list  TopFrame
        control         Control
        top_force       TopForce
        scope_list      MyScope
        myTenseMath     TenseMath

        %Jméno systému - public pro nastavení listeneru
        system_name = 'Vygenerovana_tensegrita'
    end
    properties(Access = private, Constant)
        %Parametry simulace
        stop_time = inf;
        OPEN_SYSTEM = 1;
        RelTol = '1e-3'
        MaxStep = '5e-1'
        SolverType = 'daessc'
        ScopeSampleRate = '0.1'
        steady_state_speed = 1e-4
        minimal_steady_state_time = 2e-4

        %Parametry pro získání ustálených časů
        parallel_spring_tension_constant = 1 %zkrácení paralelních pružin
        %pozor par parallel_spring_tension_constant je bugnutá
        cable_tension_constant = 0.95
        initial_top_force = zeros(6,1)

        %Konstanty pro vzhled vygenerované struktury
        tensegrity_position =   [300, 300]
        tensegrity_size =       [200, 100]
        frame_position =        [0, 0]
        frame_size =            ceil([982,640]/3)
        bars_initial_position = [0, 300]
        bars_displacement =     [0, 150]
        cables_initial_position = [500, 0]
        cables_displacement =   [260, 0]

        %Jména systému
        subsystem_name = 'Tensegrita'
        StartFnc_file_name = 'MyStartFnc';

    end

    %Properties pro simulaci
    properties(Access = private)
        previous_steady_state_time

        sequence_type

        phase
        IS_TESTING

        cable_forces
        wanted_nodes
        fixed_nodes
    end
    properties(Access = public)
        actual_simulation_time
        time_vector
        time_index
        end_efector_initial_point
        enf_efector_initial_rotation
        end_efector_position
        end_efector_velocity
        end_efector_angles
        end_efector_axis

        nodes_actual

        relative_position
        relative_rotation

        free_lengths
    end
    methods(Access = public)
        function obj = GeneratorV4(varargin)
            %konstruktor přepíše parametry modelu
            obj@TensegritySettings(varargin)
        end
        function buildModel(obj)
            %Vyplnění dat, vytvoření systémů
            obj.createSystem();
            obj.createSubsystem();
            obj.setStartUpFunction();
            obj.fillBars();
            obj.fillCables();
            %             obj.fillJoints();        %Neimplementováno
            obj.fillMuxes();
            obj.fillDemuxes();
            obj.fillTopFrame();
            obj.fillBotFrame();
            obj.fillControl();
            obj.fillScopes();
            obj.fillTopForce();

            obj.connectMuxes();
            obj.connectDemuxes();
            obj.connectCables();
            obj.connectFrames();
            obj.connectControl();
            obj.connectScopes();
            obj.connectTopForce();

            obj.myTenseMath = TenseMath(obj.bars, obj.cables, obj.frames, obj.nodes);


        end
        function startSimulation(obj)
            %Vyresetování všech stavů
            obj.previous_steady_state_time = 0;
            obj.phase = 0;
            obj.IS_TESTING = 0;
            obj.time_vector = 0;
            obj.time_index = 1;

            sim(obj.system_name);
        end
        function obj = controlFunction(obj, varargin)
            %% Sbírání dat
            %time_vector je vektor obsahující jednotlivé časové kroky
            %time_index obsahuje číslo udávající počet uplynulých kroků
            actual_time = get_param(obj.system_name, 'SimulationTime');
            obj.actual_simulation_time = actual_time;
            obj.time_vector(obj.time_index) = actual_time;

            %Jména bloků ze kterých se čtou informace
            block1_path = obj.scope_list(1).path;
            block2_path = obj.scope_list(2).path;
            block3_path = obj.scope_list(3).path;

            %Řešení sil v lanech scope 1
            scope1_data = get_param(block1_path,'RuntimeObject').InputPort(1).Data;
            obj.cable_forces(obj.time_index,:) = scope1_data;

            %Řešení poloh uzlů scope 2 - aktuální polohy uzlů
            scope2_data = get_param(block2_path,'RuntimeObject').InputPort(1).Data;
            obj.nodes_actual = scope2_data;
            obj.nodes_actual = obj.mapNodes(scope2_data);

            %Čtení dat ze scopu 3 - scope natočení end-efektoru
            scope3_data = get_param(block3_path,'RuntimeObject').InputPort(1).Data;
            rotation_matrix = reshape(scope3_data(1:9),[3,3]);
            obj.end_efector_angles(obj.time_index, :) = rotm2eul(rotation_matrix);
            obj.end_efector_axis(obj.time_index, :) = rotm2axang(rotation_matrix);
            obj.end_efector_position(obj.time_index,:) = scope3_data(9+(1:3));
            obj.end_efector_velocity(obj.time_index,:) = scope3_data(13);

            %Přenastavení délky lana
            if obj.time_index == 1
%                 l01 = obj.myTenseMath.freeLength(obj.nodes_actual);
%                 l02 = obj.myTenseMath.freeLength2(obj.nodes_actual);
%                 obj.control.setFreeLength(l01);
%                 obj.myTenseMath.cables.free_lengths_cables
            end

            %% Řízení
            if(obj.time_index > 10 && ...
                    norm(obj.end_efector_velocity(end-9, end)) <= obj.steady_state_speed && ...
                    obj.actual_simulation_time - obj.previous_steady_state_time > 0.2)
                %Uložení nového času ustáleného stavu
                obj.previous_steady_state_time = obj.actual_simulation_time;
                %Ustálený stav dosažen - něco se bude dít
                try
                    switch obj.phase
                        case 0
                            %Fáze 0 je výběr ustáleného bodu
                            %Střed plošiny
                            obj.end_efector_initial_point = mean(obj.nodes_actual(:,[10,11,12]),2);
                            obj.fixed_nodes = zeros(3,12);
                            obj.fixed_nodes(:, 1:6) = obj.nodes_actual(:, 1:6);
                            obj.fixed_nodes(:, 7:12) = obj.nodes_actual(:, 7:12) - obj.end_efector_initial_point;

                            obj.relative_position = zeros(3,1);
                            obj.relative_rotation = zeros(3,1);
                        case 1
                            obj.moveTo2([0,0,0], [0,0,0])
                        otherwise
                            try
                                x = 0.0025 * sin(2*pi*obj.phase/40);
                                y = 0;
                                if obj.phase > 30
                                    y = 0.000 * sin(2*pi*(obj.phase-20)/60);
                                end
                                obj.moveTo2([x,y,0], [0,0,0], 0)
                            catch exception_inner
                                disp(exception_inner.message)
                            end

%                             obj.moveTo([0.00,0.001,0.0], [0.00,0,0],1)
                    end
                    obj.phase = obj.phase +1;
                catch exception
                    disp("Something went wrong: "+exception);
                end
                %Krok 1 - nastavení výchozí polohy
                if actual_time > 50
                    obj.stopSimulation();
                end 
            end
            obj.time_index = obj.time_index+1;
        end
    end

    methods(Access = private)
        %Nastavení systému
        function createSystem(obj)
            %Pokud je systém už vytvořený, smaže starý
            if isempty([obj.system_name,'.slx'])
                new_system(obj.system_name)
            else
                close_system(obj.system_name,0)
                delete([obj.system_name,'.slx'])
                new_system(obj.system_name)
            end
            %Nastaví parametry simulace
            set_param(obj.system_name, 'StopTime', num2str(obj.stop_time))
            set_param(obj.system_name, 'RelTol', obj.RelTol)
            set_param(obj.system_name, 'Solver', obj.SolverType)
            set_param(obj.system_name, 'MaxStep', obj.MaxStep)
            %Pokud je potřeba otevře se na začátku
            if obj.OPEN_SYSTEM
                open_system(obj.system_name)
                set_param(gcs,'Location',get(0,'ScreenSize'));
            end
            %Systém je potřeba uložit - tím se soubor vytvoří
            save_system(obj.system_name)
        end
        function createSubsystem(obj)
            %Metoda vytvoří subsystém, ve kterém se bude struktura
            %modelovat, smaže vnitřní porty
            subsystem_path = obj.path(obj.subsystem_name);
            add_block('simulink/Ports & Subsystems/Subsystem',...
                subsystem_path,...
                'Position', obj.position(obj.tensegrity_position, obj.tensegrity_size))
            delete_line(subsystem_path,'In1/1','Out1/1')
            delete_block(obj.path('In1', subsystem_path));
            delete_block(obj.path('Out1', subsystem_path));
            open_system(subsystem_path)
        end
        function setStartUpFunction(obj)
            set_param(obj.system_name, 'StartFcn', obj.StartFnc_file_name);
        end
        
        %Inicializace a přidání bloků
        function fillBars(obj)
            %bars_position obsahuje pozici kam se bude tyč generovat
            bars_position = obj.bars_initial_position;
            for i = 1:obj.bars.count
                obj.bars_list(i) = Bar(...
                    bars_position,...
                    i, ...
                    gcs, ...
                    obj.bars.diameters(i), ...
                    obj.bars.lengths(i), ...
                    obj.bars.mid_points(:,i), ...
                    obj.bars.rotation_matrixes(:,:,i));
                %Tyč rovnou i přidá
                obj.bars_list(i).addBar();
                bars_position = bars_position + obj.bars_displacement;
            end
            obj.end_bars_position = bars_position;
        end
        function fillCables(obj)
            %cables_position obsahuje pozici kam se bude tyč generovat
            cables_position = obj.cables_initial_position;
            for i = 1:obj.cables.count
                obj.cables_list(i) = Cable(cables_position,...
                    i, ...
                    gcs, ...
                    obj.cables.specific_stiffness(i), ...
                    obj.cables.specific_dumpings(i), ...
                    obj.cables.stiffness(i), ...
                    obj.cables.dumpings(i),...
                    obj.cables.free_length(i)*obj.parallel_spring_tension_constant);
                %Tyč rovnou i přidá
                obj.cables_list(i).addCable();
                cables_position = cables_position + obj.cables_displacement;
            end
            obj.end_cables_position = cables_position;
        end
        function fillJoints(obj)
            %Neimplementováno z důvodu, že generovaná struktura neobsahuje
            %klouby mezi jednotlivými tyčemi - pouze mezi rámy
            %funkce:
            %1) najde duplicitní uzly - tj ty, které obsahuje vícero tyčí
            %             nodes = obj.bars.from_to(:);
        end
        function fillMuxes(obj)
            obj.mux_list(1) = Mux(obj.end_cables_position, 1, gcs,obj.cables.count, 'Forces');
            obj.mux_list(1).addMux();
            obj.mux_list(2) = Mux(obj.end_bars_position, 2, gcs,obj.bars.count, 'Positions');
            obj.mux_list(2).addMux();
        end
        function fillDemuxes(obj)
            obj.demux_list(1) = Demux([150,-100],1, gcs, obj.cables.count);
            obj.demux_list(1).addDemux();
        end
        function fillTopFrame(obj)
            obj.top_frame_list(1) = TopFrame([1000, 1000], 1, gcs, obj.frames.radius_top, obj.frames.width_top, ...
                [obj.nodes(1:2,end-2:end); zeros(1,3)]); % posun od středu horního kusu jen v X a Y - v Z je to 0
            obj.top_frame_list(1).addTopFrame();
        end
        function fillBotFrame(obj)
            obj.bot_frame_list(1) = BotFrame([0, 0], 1, gcs, obj.frames.radius_bot, obj.frames.width_bot, obj.nodes(1:3,1:3));
            obj.bot_frame_list(1).addBotFrame();
        end
        function fillControl(obj)
            obj.control(1) = Control([0, 0], 1, obj.system_name, obj.cables.free_length*obj.cable_tension_constant);
            obj.control(1).addControl();

            %Nastavení volných délek pro pevná lana
            obj.cables.free_lengths_cables = obj.cables.free_length*obj.cable_tension_constant;
        end
        function fillScopes(obj)
            %Scope pro síly
            obj.scope_list(1) = MyScope(obj.tensegrity_position + [250, -100], ...
                1,...
                obj.system_name,...
                'Síly', ...
                obj.ScopeSampleRate);
            obj.scope_list(1).addScope();

            %Scope pro pozice tyčí
            obj.scope_list(2) = MyScope(obj.tensegrity_position + [250, 000], ...
                2,...
                obj.system_name,...
                'Pozice tyčí');
            obj.scope_list(2).addScope();

            %Scope pro polohu end efektoru
            obj.scope_list(3) = MyScope(obj.tensegrity_position + [250, 100], ...
                3,...
                obj.system_name,...
                'End efektor');
            obj.scope_list(3).addScope();
        end
        function fillTopForce(obj)
            obj.top_force(1) = TopForce(obj.tensegrity_position + [300, 200], ...
                1, ...
                obj.system_name, obj.initial_top_force);
            obj.top_force.addTopFrame();
        end

        %Pospojování bloků
        function connectCables(obj)
            %i - aktuálně řešené lano
            %hledají se dvě tyče, se kterými je třeba spojit
            for i = 1:obj.cables.count
                %j - pravý / levý port
                for j = 1:2
                    %node - uzel příslušející i-tému lanu j-té straně
                    node = obj.cables.from_to(i,j); %číslo uzlu
                    %r, c, řádek/sloupec pro příslušnou tyč, která má
                    %stejný uzel jako daný konec lana
                    [r, c] = find(obj.bars.from_to == node);
                    c = c(1); %stačí to připojit k jedné tyči - řeší problém kdy by se mělo přidat na místo kde je kloub
                    if ~mod(j,2)
                        port1 = '/Rconn1';
                    else
                        port1 = '/Lconn1';
                    end
                    if ~mod(c,2)
                        port2 = '/Rconn1';
                    else
                        port2 = '/Lconn1';
                    end
                    cable_path = [obj.cables_list(i).blockName, port1];
                    bar_path = [obj.bars_list(r(1)).blockName, port2];
                    add_line(gcs, bar_path, cable_path);
                end
            end
        end
        function connectMuxes(obj)
            for i = 1:obj.cables.count
                cable_path = [obj.cables_list(i).blockName,'/1'];
                mux_path = [obj.mux_list(1).blockName, '/', num2str(i)];
                add_line(gcs, cable_path, mux_path)
            end
            for i = 1:obj.bars.count
                bar_path = [obj.bars_list(i).blockName,'/1'];
                mux_path = [obj.mux_list(2).blockName, '/', num2str(i)];
                add_line(gcs, bar_path, mux_path)
            end
        end
        function connectDemuxes(obj)
            for i = 1:obj.cables.count
                demux_path = [obj.demux_list(1).blockName, '/', num2str(i)];
                cable_path = [obj.cables_list(i).blockName,'/1'];
                add_line(gcs, demux_path, cable_path);
            end
        end
        function connectFrames(obj)
            %Spojení spodního rámu
            for i = 1:3
                add_line(gcs,...
                    [obj.bot_frame_list(1).blockName, '/Rconn', num2str(i)],...
                    [obj.bars_list(i).blockName, '/Lconn1'])
            end
            %Spojení horního rámu
            for i = 1:3
                add_line(gcs,...
                    [obj.top_frame_list(1).blockName, '/Lconn', num2str(i)],...
                    [obj.bars_list(end+i-3).blockName, '/Rconn1'])
            end
        end
        function connectControl(obj)
            add_line(obj.system_name, ...
                [obj.control(1).blockName, '/1'], ...
                [obj.subsystem_name, '/1'])
        end
        function connectScopes(obj)
            for i = 1:numel(obj.scope_list)
                add_line(obj.system_name, ...
                    [obj.subsystem_name, '/',num2str(i)], ...
                    [obj.scope_list(i).blockName, '/1'])
            end
        end
        function connectTopForce(obj)
            add_line(obj.system_name,...
                [obj.subsystem_name, '/Rconn1'], ...
                [obj.top_force.blockName, '/Lconn1'])
        end
    end
    %Metody pro simulace
    methods(Access = private)
        function stopSimulation(obj)
            set_param(obj.system_name, 'SimulationCommand', 'stop')
        end
        function mapped_nodes = mapNodes(obj, nodes)
            %Přemapuje uzly z toho co se nasbíralo z modelu na normální
            %3x12 vektor
            from_to_vec = obj.bars.from_to';
            from_to_vec = from_to_vec(:);
            mapped_nodes = zeros(3,12);
            for i = 1:numel(from_to_vec)
                map = from_to_vec(i);
                mapped_nodes(:,map) = nodes((i-1)*3+(1:3));
            end
        end
        function moveTo(obj, position, rotation, isRelativeMovement)
            %Posune end efektor tensegrity na požadovanou polohu a
            %požadované natočení dané kardanovo úhly
            %Při isRelativeMovement = 1 je pozice a rotace přičítána
            %Používá pro řízení všechna lana, zachovává pevnou pozici
            position = reshape(position, [],1);
            rotation = reshape(rotation, [],1);
            switch nargin
                case 3
                    obj.relative_position = position;
                    obj.relative_rotation = rotation;
                case 4
                    if isRelativeMovement == 0
                        obj.relative_position = position;
                        obj.relative_rotation = rotation;
                    else
                        obj.relative_position = obj.relative_position+position;
                        obj.relative_rotation = obj.relative_rotation+rotation;
                    end
                otherwise
                    error("Špatný počet vstupů");
            end

            %N_new obsahuje nové uspořádání uzlů
            N_new = obj.fixed_nodes;

            %Je potřeba horní uzly přetrasformovat tak, aby end-efektor byl
            %správně
            N_new(:, 7:end) = obj.myTenseMath.TransformMatrix(N_new(:, 7:end), ...
                obj.end_efector_initial_point + obj.relative_position, ...
                obj.relative_rotation);
            l0 = obj.myTenseMath.freeLength(N_new);
            obj.control.setFreeLength(l0);
        end
        function moveTo2(obj, position, rotation, isRelativeMovement)
            %Posune end efektor tensegrity na požadovanou polohu a
            %požadované natočení dané kardanovo úhly
            %Při isRelativeMovement = 1 je pozice a rotace přičítána
            position = reshape(position, [],1);
            rotation = reshape(rotation, [],1);
            switch nargin
                case 3
                    obj.relative_position = position;
                    obj.relative_rotation = rotation;
                case 4
                    if isRelativeMovement == 0
                        obj.relative_position = position;
                        obj.relative_rotation = rotation;
                    else
                        obj.relative_position = obj.relative_position+position;
                        obj.relative_rotation = obj.relative_rotation+rotation;
                    end
                otherwise
                    error("Špatný počet vstupů");
            end

            %N_new obsahuje nové uspořádání uzlů - pro malé změny se
            %jako počáteční odhad použijí uzly, ve kterých se struktura
            %nachází
            disp("Relativni poloha x: "+obj.relative_position(1))
            disp("Relativni poloha y: "+obj.relative_position(2))
            N_new = obj.nodes_actual;
            N_new(:,end-2:end) = N_new(:,end-2:end) + repmat(position, 1,3);
            
            %Je potřeba horní uzly přetrasformovat tak, aby end-efektor byl
            %správně
            l0 = obj.myTenseMath.freeLength2(N_new);
            obj.control.setFreeLength(l0);
        end
    end

    %Pomocné metody
    methods(Access = private)
        function position = position(~, varargin)
            %Vrátí sting pro vyplnění pozice při generování bloků
            %arg1 = pozice bloku
            %arg2 = velikost bloku (default [100 100])
            switch nargin
                case 2
                    block_position = varargin{1};
                    block_size = [100, 100];
                case 3
                    block_position = varargin{1};
                    block_size = varargin{2};
            end
            position = mat2str([block_position, block_position+block_size]);
        end
        function path = path(~, varargin)
            %Vrátí cestu k bloku,
            %arg1 = jméno bloku
            %arg2 = adresa systému / subsytému k bloku
            switch nargin
                case 2
                    path = [gcs, '/', varargin{1}];
                case 3
                    path = [varargin{2}, '/', varargin{1}];
            end
        end
    end
end