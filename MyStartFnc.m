%% Sekvence provedená při startu modelu - hlavně jde o nastavení listenerů
%na příslušné bloky
%Bloky, do kterých se vloží event listener
block1 = [myGen.system_name, '/', myGen.scope_list(1).blockName];

%Typ eventu pro trigger listener funkce
event = 'PostOutputs';

%Funkce vykonaná při eventu
listener = @myGen.controlFunction;

%Samotné nastavení listenerů
h1 = add_exec_event_listener(block1, event, listener);
% h2 = add_exec_event_listener(block2, event, listener);


