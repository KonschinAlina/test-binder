:- module(queries, [object_pose_1/2]).
:- use_module(library(janus)).

% Python-Modul importieren
% :- py_call('func_lib', [object_pose]).

% Lade Python-Modul
% :- py_module('func_lib', Lib).

% call object_pose func from lib 
object_pose_1(Object, Pose) :-
    atom(Object),
    py_call(func_lib:object_pose1, [Object], Pose).
    
% format(string(Cmd), "from my_lib import object_pose; object_pose('~w')", [Object]),
% py_call(Cmd, Pose).

% py_call(object_pose(Object), Pose).
% py_call("from func_lib import object_pose; object_pose('" + Object + "')", Pose).