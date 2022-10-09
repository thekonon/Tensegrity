classdef TenseMath < matlab.mixin.SetGet

    properties
        %Vstupy
        bars
        cables
        frames
        
        %Pro snažší zápis
        F
        lop
        ks
        Kp
        l0
        
        %Aktuální zatížení
        forces
        torques %neimplemetnováno

        %Dodané / vypočítané veličiny
        nodes               %Matice uzlů
        reduces_nodes       %Redukovaná matice uzlů o ty pevné
        fixed_nodes_indexes %Indexy pevných uzlů
        reduces_nodes_indexes   %Indexy všech uzlů až na pevné
        nodes_count
        Cb
        Cb_red
        Cr
        Cs
        Cs_red
        Gamma
        Lambda
        W
        W_reduced
        W_c
        W_c_reduced
        Wf
    end

    properties(Access = private)
        N %Aktuální matice uzlů
    end

    methods(Access = public)
        function obj = TenseMath(varargin)
            %Vytvoří objekt pro statické počítání s tensegritami
            %Vstupy:
            %1 - struktura tyčí
            %2 - struktura lan
            %3 - struktura rámu
            %Načtení vstupů
            obj.bars = varargin{1};
            obj.cables = varargin{2};
            obj.frames = varargin{3};

            obj.inicialization();
            obj.calculateAllRed(varargin{4})
        end
        function inicialization(obj)
            %Volné délky paraleleních
            obj.lop = obj.cables.free_length;
            obj.Kp = obj.cables.stiffness;
            obj.ks = obj.cables.specific_stiffness;

            %Redukce matice uzlů
            obj.fixed_nodes_indexes = find(obj.frames.fixed_nodes==1);
            obj.reduces_nodes_indexes = setdiff(1:12, obj.fixed_nodes_indexes);

            %Vyplnění informací ze vstupů
            obj.nodes_count = obj.frames.nodes_count;
            obj.Cb = obj.fromTo2C(obj.bars.from_to_extended, obj.nodes_count);
            obj.Cr = abs(obj.Cb);
            obj.Cs = obj.fromTo2C(obj.cables.from_to, obj.nodes_count);
            obj.Cb_red = obj.Cb(:,obj.reduces_nodes_indexes);
            obj.Cs_red = obj.Cb(:,obj.reduces_nodes_indexes);
            
            %Vytvoření vektoru zatížení
            obj.W = zeros(3, obj.nodes_count);
            obj.W(3,:) = -9.81/2*0.023*ones(1,9)*obj.Cr;
            obj.W_reduced = obj.W(:, obj.reduces_nodes_indexes);

            %Převod do vektorové podoby
            obj.W_c = obj.W(:);
            obj.W_c_reduced = obj.W_reduced(:);
        end
        function l0 = freeLength(obj, N)
            %Vzdálenosti mezi uzly
            l = sqrt(sum((N*obj.Cs').^2));

            %Potřebné síly v jednotlivých lanech
            F = sqrt(sum(obj.calculateAllRed(N).^2));
            lop = obj.cables.free_length;
            ks = obj.cables.specific_stiffness;
            Kp = obj.cables.stiffness;
            l0 = zeros(numel(l),1);
            for i = 1:numel(l)
                dlpi = (l(i)-lop(i));
                if dlpi < 0
                    dlpi = 0;
                end
                l0(i) = ks(i)*l(i)/(ks(i) - Kp(i)*dlpi + F(i));
            end
            l0 = reshape(l0, 1, []);
        end
        function l0 = freeLength2(obj, N)
            %chce to implementovat:
            %počáteční odhad pro uzly + silové hustoty
            % ->zkusit jestli to vrátí nulu (nebo zkonverguje) pro známé
            % hodnoty
            obj.N = N;

            %udělat smyčku kdy ze známé konfigurace se chci pohnout o kus
            %end efektorem, fsolvem najít nejbližší řešení

            %Vstupem je poloha x a y, z se musí dopočítat
            N_mid = N(1:2,4:9);

            options = optimoptions('fsolve', 'Display', 'off', 'FunctionTolerance', 1e-8, ...
                'MaxFunctionEvaluations', 1e9);
            [N_mid_opt, exit_val, flag] = fsolve(@obj.minFcn2, N_mid, options);
            disp("Chyba řešení: "+norm(exit_val))
            if flag <= 0
                error("Nenalezeno řešení - fsolve,"+"Flag: "+flag)
            else
                l0_prev = freeLength(obj, obj.N);
                obj.N(1:2, 4:9) = N_mid_opt;
                obj.N(3, 4:9) = obj.addNewHeights(N_mid_opt);
                l0 = freeLength(obj, obj.N);
                if norm(l0_prev - l0) > 0.1
                    l0 = l0_prev;
                end
            end
        end
        function setForces(obj, forces, torques)
            obj.forces = forces;
            obj.torques = torques;
            obj.Wf = zeros(3,obj.nodes_count);

            %Přidaná extra tíha - z horního bloku W-forces
            obj.Wf(1,end-2:end) = forces(1)*[1,1,1]/3;
            obj.Wf(2,end-2:end) = forces(2)*[1,1,1]/3;
            obj.Wf(3,end-2:end) = forces(3)*[1,1,1]/3;
            dummy_W = obj.Wf + obj.W;
            obj.W_c = dummy_W(:);
        end
    end
    methods(Access = public)
        %Manipulace se souřadnicemi
        function V  = TransformVector(obj, vektor, posun, rotace)
            %Posun je sloupcový vektor
            %Vektor je sloupcový vektor nerozšířený
            T = obj.Txyzp(posun, rotace);
            W = T*[vektor;1];
            V = W(1:3);
        end
        function V = TransformMatrix(obj, matice, posun, rotace)
            %Posun je sloupcový vektor
            %matice obsahuje sloupcové vektor nerozšířené
            T = obj.Txyzp(posun, rotace);
            rozsizena_matice = [matice; ones(1, size(matice,2))];
            W = T*rozsizena_matice;
            V = W(1:3,:);
        end
    end
    methods(Access = private)
        %Výpočty sil
        function [Fs, Fb, gamma, lambda, R] = calculateAll(obj, nodes)
            %Vypočítá síly v tyčích, lanech a příslušné silové hustoty +
            %reakce

            %Výpočet V*diag(X) = vec(W)
            Res = nodeMatrix(obj, nodes)\obj.W_c;

            %Rozdělení výsledků na dílčí výsledky
            gamma = Res(1:18);      % silové hustoty lan
            lambda = Res(19:27);    % silové hustoty tyčí
            R = Res(28:end);        % reakce

            %Dopočet skutečných sil v lanech
            Fs = nodes*obj.Cs'*diag(gamma);
            Fb = nodes*obj.Cb'*diag(lambda);
        end
        function [Fs, Fb, gamma, lambda] = calculateAllRed(obj, nodes)
            %Vypočítá síly v tyčích, lanech a příslušné silové hustoty v
            %redukovaných uzlech - bez reakcí

            %Výpočet V*diag(X) = vec(W)
            [~, V_red] = obj.nodeMatrix(nodes);
            Res = V_red\obj.W_c_reduced;

            %Rozdělení výsledků na dílčí výsledky
            gamma = Res(1:18);      % silové hustoty lan
            lambda = Res(19:27);    % silové hustoty tyčí

            %Dopočet skutečných sil v lanech
            Fs = nodes*obj.Cs'*diag(gamma);
            Fb = nodes*obj.Cb'*diag(lambda);
        end
        function [V, V_red] = nodeMatrix(obj, N)
            %Převod rovnice A*X*B = -W do vektoré podoby
            %Kde X = diag([x1, x2, x3...xn])
            %Odvození v mé práci
            A = [N*obj.Cs', N*obj.Cb']; %#ok<*PROPLC>
            B = [obj.Cs; obj.Cb];

            %Výsledná matice V vystupující v rovnici
            %V*diag(X) = -vec(W)
            V = (zeros(36));

            %Odvození pro for cyklus v přiložené literatuře
            for i = 1:size(A,2)
                ai = A(:,i);
                bi = B(i,:);
                V(:,i) = kron(bi',ai);
            end
            ind = repmat(obj.frames.fixed_nodes,3,1);
            ind = ind(:);
            V_red = V(:,1:end-9);
            V_red(ind==1, :) = [];
        end
        function residuum_vector = minFcn(obj, N_mid)
            %Nové uzly - které jsou typnuty se poskládají do matice
            N_new = obj.N;

            %Přepsání natipovaných uzlů
            N_new(1:2, 4:9) = N_mid;

            %Dopočet souřadnice Z
            for i = 1:obj.bars.count
                if i <= 3
                    N_new(3, 3+i) = N_new(3, i) + sqrt(obj.bars.lengths(i)^2 - sumsqr(diff(N_new(1:2,obj.bars.from_to(i,:))')));
                else
                    res = N_new(3, 6+i) - sqrt(obj.bars.lengths(i)^2 - sumsqr(diff(N_new(1:2,obj.bars.from_to(i,:))')));
                    N_new(3, 3+i) = res;
                end
            end

            %Získá se silová hustata vzniklá při dané uzlové konfiguraci -
            %ta se bude porovnávat se skutečnou
            [~, ~, Gamma, ~] = obj.calculateAllRed(N_new);

            %Dále je třeba vypočítat skutečnou při dané konfiguraci
            l = sqrt(sum((N_new*obj.Cs').^2));
            l0 = obj.cables.free_lengths_cables;
            
            %Výpočet skutečné gammy
            my_gamma = zeros(numel(obj.cables.fixed_cables_indexes),1);
%             my_FS = zeros(numel(obj.cables_structure.fixed_cables_indexes),1);
            for i = obj.cables.fixed_cables_indexes
                dlp = (l(i)-obj.lop(i));
                if dlp < 0
                    dlp = 0;
                end
                dl = (l(i)-l0(i));
                if dl < 0
                    dl = 0;
                end
                my_gamma(i) = (obj.ks(i)/l0(i)*dl + obj.Kp(i)*dlp)/l(i);
%                 my_FS = (obj.ks(i)/l0(i)*dl + obj.Kp(i)*dlp);
            end
            
            %Použití jen gammy pro pevné uzly
            Gamma = Gamma(obj.cables.fixed_cables_indexes);
            my_gamma = my_gamma(obj.cables.fixed_cables_indexes);
            %Jejich rozdíl
            residuum_vector = my_gamma - Gamma;
%             residuum_vector = Fs-my_FS;
        end
        function out = minFcn2(obj, N_mid)
            %Nové uzly - které jsou typnuty se poskládají do matice
            N_new = obj.N;

            %Přepsání natipovaných uzlů
            N_new(1:2, 4:9) = N_mid;

            %Dopočet souřadnice Z
            for i = 1:obj.bars.count
                if i <= 3
                    N_new(3, 3+i) = N_new(3, i) + sqrt(obj.bars.lengths(i)^2 - sumsqr(diff(N_new(1:2,obj.bars.from_to(i,:))')));
                else
                    res = N_new(3, 6+i) - sqrt(obj.bars.lengths(i)^2 - sumsqr(diff(N_new(1:2,obj.bars.from_to(i,:))')));
                    N_new(3, 3+i) = res;
                end
            end

             %Porovnání volných délek
                l0_new = obj.freeLength(N_new);
             difference = l0_new - obj.cables.free_lengths_cables;

             %Zkoumají se pouze fixnuté lana
             difference = difference(obj.cables.fixed_cables_indexes);
             out = difference;
        end
        function N_z = addNewHeights(obj, N_mid)
            %Nové uzly - které jsou typnuty se poskládají do matice
            N_new = obj.N;

            %Přepsání natipovaných uzlů
            N_new(1:2, 4:9) = N_mid;

            %Dopočet souřadnice Z
            for i = 1:obj.bars.count
                if i <= 3
                    N_new(3, 3+i) = N_new(3, i) + sqrt(obj.bars.lengths(i)^2 - sumsqr(diff(N_new(1:2,obj.bars.from_to(i,:))')));
                else
                    res = N_new(3, 6+i) - sqrt(obj.bars.lengths(i)^2 - sumsqr(diff(N_new(1:2,obj.bars.from_to(i,:))')));
                    N_new(3, 3+i) = res;
                end
            end
            N_z = N_new(3, 4:9);
        end

        %Manipulace se souřadnicemi
        function W = addUp(obj, konstantni_vektor, vektor_rotace, rotace)
            W = konstantni_vektor + obj.S12(rotace)*vektor_rotace;
        end
        function vec = e(~,i,n)
            vec = zeros(n,1);
            vec(i) = 1;
        end
        function C = fromTo2C(obj, from_to, nodes_count)
            n = size(from_to,1);    %Celkem prvků
            m = nodes_count;        %Celkem uzlů
            C = zeros(n,m);
            for i = 1:n
                C(i,:) = obj.e(from_to(i, 2), m) - obj.e(from_to(i, 1), m);
            end
        end
        function S = S12(obj, rotace)
            S = obj.Rx(rotace(1))*obj.Ry(rotace(2))*obj.Rz(rotace(3));
        end
        function T = Txyzp(obj, posun, rotace)
            T = [obj.Rx(rotace(1))*obj.Ry(rotace(2))*obj.Rz(rotace(3)),posun;
                zeros(1,3), 1];
        end
        function Rx = Rx(~, phi)
            Rx = [1 0 0;...
                0 cosd(phi) -sind(phi);...
                0 sind(phi) cosd(phi)];
        end
        function Ry = Ry(~, phi)
            Ry = [cosd(phi) 0 sind(phi);...
                0 1 0;...
                -sind(phi) 0 cosd(phi)];
        end
        function Rz = Rz(~, phi)
            Rz =[cosd(phi), -sind(phi), 0;...
                sind(phi), cosd(phi), 0; ...
                0, 0, 1];
        end
    end
end

