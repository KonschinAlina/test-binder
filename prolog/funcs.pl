:- module(queries, 
          [ object_pose/3,
            highlight_object/4,
            highlight_object_list/4,
            trajectory/4,
            highlight_trajectory/3
            ]).
          
:- use_module(library(janus)).

% :- use_module(library('ros/tf/tf')).
% :- use_module(library('model/RDFS'), [ has_type/2 ]).

% Creates an Instance of the class FuncLib
% :- py_call('func_lib.FuncLib'(), [], Obj).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% object_pose(+ClassInstance, +ObjectFrame, -Pose)
%
object_pose(ClassInst, ObjectFrame, Pose) :-
    atom(ObjectFrame),
    py_call(ClassInst:object_pose(ObjectFrame), Pose).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% highlight_object(+ClassInstance, +XacroPath, +ObjectName, -HighLight)
%
highlight_object(ClassInst, XP, ObjectName, Highlight):-
    atom(ObjectName),
    py_call(ClassInst:highlight(XP, ObjectName), Highlight).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% highlight_object_list(+ClassInstance, +XacroPaths, +ObjectNames, -Highlight)
%
highlight_object_list(ClassInst, XPs, ObjectList, Highlight):-
    py_call(ClassInst:highlight_list(XPs, ObjectList), Highlight).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% trajectory(+ClassInst, +Action, +PointList, -Trajectory)
%
trajectory(ClassInst, Action, PointList, Trajectory):-
    py_call(ClassInst:trajectory(Action, PointList), Trajectory).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% highlight_trajectory(+ClassInst, +PointList,-Highlight)
%
highlight_trajectory(ClassInst, PointList, Highlight):-
    py_call(ClassInst:highlight_trajectory(PointList), Highlight).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% findall(Schrank, schrank_typ(Schrank, X), Result), highlight(Result).


