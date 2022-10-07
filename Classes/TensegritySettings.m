classdef TensegritySettings < matlab.mixin.SetGet
    %Třída obsahuje základní parametry úlohy

    properties
        bars
        cables
        frames
        nodes
    end

    methods
        function obj = TensegritySettings(varargin)
            %% Nastaví hodnoty společné pro všechny úlohy
            obj.inicialization();
            %% Možnost přepsání hodnot, které jsou například optimalizovány
            %Takto by vypadalo přepsání tuhosti lana
            %první argument - index lana
            %druhý - jeho měrná tuhost
            %obj.cables.specific_stiffness(varargin{1}) = varargin{2};
            %% Zde se ze zadaných hodnot dopočítají polohy uzlů
            obj.initialNodesGuess();
            
            %Délky lan je třeba dopočítat z původního odhadu (délky
            %paralelních pružin například)
            coresponding_nodes_cables = obj.nodes(:,obj.cables.from_to');
            obj.cables.free_length = sqrt(sum((coresponding_nodes_cables(:,1:2:end)-coresponding_nodes_cables(:,2:2:end)).^2,1));
            load steady_state_nodes.mat nodes
            obj.nodes = nodes'; %Které je možné přepsat třeba přesnějším
%                                     řešením
            %% Dopočet počátečních souřadnic
            %tj natočení tyčí, polohy středů pro SimScape
            obj.calculateInitialCoordinates();

        end
    end
    methods(Access = private)
        function inicialization(obj)
            %Nastaví pevné parametry společné pro všechny úlohy
            obj.setBars();
            obj.setCables();
            obj.setFrames();
        end
        function setBars(obj)
            obj.bars.count = 6;
            obj.bars.lengths =      ones(obj.bars.count,1)*0.3;
            obj.bars.diameters =    ones(obj.bars.count,1)*0.01;
            obj.bars.densitys =     ones(obj.bars.count,1)*1000;
            obj.bars.from_to = [1 4;...
                2 5
                3 6
                7 11
                8 12
                9 10];
            obj.bars.from_to_extended = [1 4;...
                2 5
                3 6
                7 11
                8 12
                9 10
                10 11
                11 12
                10 12];
        end
        function setCables(obj)
            obj.cables.count = 18;
            obj.cables.specific_stiffness = ones(obj.cables.count,1)*4000;
            obj.cables.specific_dumpings = ones(obj.cables.count,1)*400;
            obj.cables.stiffness = ones(obj.cables.count,1)*25;
            obj.cables.dumpings = ones(obj.cables.count,1)*10;
            obj.cables.variable_cables_indexes = [7,8,9,10,11,12];
            obj.cables.fixed_cables_indexes = setdiff(1:obj.cables.count, obj.cables.variable_cables_indexes,'sorted');
            obj.cables.from_to = ...
                [1 5    %1
                1 7     %2
                2 6     %3
                2 8     %4
                3 4     %5
                3 9     %6
                4 7    %7
                4 9     %8
                5 7     %9
                5 8     %10
                6 8     %11
                6 9    %12
                6 12    %13
                9 12     %14
                4 10     %15
                7 10    %16
                5 11    %17
                8 11];  %18
        end
        function setFrames(obj)
            obj.frames.radius_mid = 0.12;
            obj.frames.rotation1 = -120;
            obj.frames.rotation2 = -60;
            obj.frames.nodes_count = max([obj.bars.from_to(:);obj.cables.from_to(:)]);
            obj.frames.fixed_nodes = zeros(1,obj.frames.nodes_count);
            obj.frames.fixed_nodes([1,2,3]) = [1,1,1];
            obj.frames.layers_shift = 0.05;

            obj.frames.width_bot = 0.005;
            obj.frames.width_top = 0.005;
            obj.frames.radius_bot = 0.1;
            obj.frames.radius_top = 0.1;
            obj.frames.type = 1;
        end
        function initialNodesGuess(obj)
            obj.nodes = zeros(3,max(obj.bars.from_to(:)));
            %Bot nodes
            obj.nodes(1:2,1) = obj.frames.radius_bot*obj.angle2vector(0);
            obj.nodes(1:2,2) = obj.frames.radius_bot*obj.angle2vector(120);
            obj.nodes(1:2,3) = obj.frames.radius_bot*obj.angle2vector(240);
            %Mid nodes
            obj.nodes(1:2,4) = obj.frames.radius_mid*obj.angle2vector(0+obj.frames.rotation1);
            obj.nodes(1:2,5) = obj.frames.radius_mid*obj.angle2vector(120+obj.frames.rotation1);
            obj.nodes(1:2,6) = obj.frames.radius_mid*obj.angle2vector(240+obj.frames.rotation1);
            obj.nodes(1:2,7) = obj.frames.radius_mid*obj.angle2vector(0+obj.frames.rotation1+60);
            obj.nodes(1:2,8) = obj.frames.radius_mid*obj.angle2vector(120+obj.frames.rotation1+60);
            obj.nodes(1:2,9) = obj.frames.radius_mid*obj.angle2vector(240+obj.frames.rotation1+60);
            %Top nodes
            obj.nodes(1:2,10) = obj.frames.radius_top*obj.angle2vector(0+obj.frames.rotation2);
            obj.nodes(1:2,11) = obj.frames.radius_top*obj.angle2vector(120+obj.frames.rotation2);
            obj.nodes(1:2,12) = obj.frames.radius_top*obj.angle2vector(240+obj.frames.rotation2);

            height_first_floor = sqrt(obj.bars.lengths(1)^2 - sumsqr(diff(obj.nodes(1:2,obj.bars.from_to(1,:))')));
            height_second_floor = sqrt(obj.bars.lengths(1)^2 - sumsqr(diff(obj.nodes(1:2,obj.bars.from_to(end,:))')));

            obj.nodes(3,:) = repelem([0,...     první patro
                height_first_floor,...      druhé patro
                height_first_floor-obj.frames.layers_shift,...  podpatro
                height_first_floor+height_second_floor-obj.frames.layers_shift], 3);    %Vrchní patro
        end
        function calculateInitialCoordinates(obj)
            %Středy tyčí
            coresponding_nodes_bars = obj.nodes(:,obj.bars.from_to');
            obj.bars.mid_points = (coresponding_nodes_bars(:,1:2:end)+coresponding_nodes_bars(:,2:2:end))/2;

            %Natočení tyčí
            for i = 1:obj.bars.count
                [alpha,beta,rotation_matrix] = obj.transformationMatrix(coresponding_nodes_bars(:, 2*i-1),coresponding_nodes_bars(:, 2*i));
                obj.bars.alpha(i) = alpha;
                obj.bars.beta(i) = beta;
                obj.bars.rotation_matrixes(:,:,i) = rotation_matrix;
            end
        end
    end
    methods(Access = private) %Pomocné funkce
        function vector = angle2vector(~, angle_d)
            vector = [cosd(angle_d), sind(angle_d)];
        end
        function [alpha, beta, R] = transformationMatrix(~, N1, N2)
            dr = N2-N1;
            dx = dr(1);
            dy = dr(2);
            dz = dr(3);
            Rx = @(phi)[1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
            Ry = @(phi)[cos(phi) 0 sin(phi); 0 1 0; -sin(phi) 0 cos(phi)];

            alpha = atan2(dy,dz);
            beta = atan2(dx,sqrt(dy.^2+dz.^2));

            R1 = Rx(-alpha);
            R2 = Ry(beta);
            R = R1*R2;
        end
    end
end