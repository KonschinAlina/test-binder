:- module(queries, 
          [ object_pose/3,
            highlight_object/3,
            highlight_object_list/3,
            trajectory/4,
            highlight_trajectory/3,
            all_tables/2
            ]).
          
:- use_module(library(janus)).
:- use_module(library(semweb/rdfs)).


% :- use_module(library('ros/tf/tf')).
% :- use_module(library('model/RDFS'), [ has_type/2 ]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% object_pose(+ClassInstance, +ObjectFrame, -Pose)
%
object_pose(ClassInst, ObjectFrame, Pose) :-
    atom(ObjectFrame),
    py_call(ClassInst:object_pose(ObjectFrame), Pose).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% highlight_object(+ClassInstance, +ObjectName, -HighLight)
%
highlight_object(ClassInst, ObjectName, Highlight):-
    atom(ObjectName),
    py_call(ClassInst:highlight(ObjectName), Highlight).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% highlight_object_list(+ClassInstance, +ObjectNames, -Highlight)
%
highlight_object_list(ClassInst, ObjectList, Highlight):-
    py_call(ClassInst:highlight_list(ObjectList), Highlight).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% trajectory(+ClassInst, +Action, +PointList, -Trajectory)
%
trajectory(ClassInst, Action, PointList, Trajectory):-
    py_call(ClassInst:trajectory(Action, PointList), Trajectory).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% highlight_trajectory(+ClassInst, +PointList, -Highlight)
%
highlight_trajectory(ClassInst, PointList, Highlight):-
    py_call(ClassInst:highlight_trajectory(PointList), Highlight).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% findall(Schrank, schrank_typ(Schrank, X), Result), highlight(Result).
%:- rdf_register_prefix(knowrob, 'http://knowrob.org/kb/knowrob.owl#').
%:- rdf_register_prefix(rdfs, 'http://www.w3.org/2000/01/rdf-schema#').


%frame_to_object('iai_kitchen/popcorn_table:popcorn_table:table_center', popcorn_table).
%frame_to_object('iai_kitchen/long_table:long_table:table_center', long_table).

% rdf_assert(knowrob:'Table', rdf:type, rdfs:'Class').

% rdf_assert(popcorn_table, rdf:type, knowrob:'Table').
% rdf_assert(long_table, rdf:type, knowrob:'Table').


all_tables(ObjType, Instances):-
    findall(Inst, rdfs_individual_of(Inst, ObjType), Instances).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

