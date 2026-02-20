:- module(queries, 
          [ object_pose/3,
            highlight_object/3,
            highlight_object_list/3,
            highlight_indiv_list/3,
            trajectory/3,
            trajectory_list/3,
            open_door/3,
            opening_information/4,
            highlight_trajectory/3
            ]).
          
:- use_module(library(janus)).
:- use_module(library(semweb/rdfs)).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% object_pose(+ClassInstance, +ObjectFrame, -Pose)
%
%% Returns the pose of an object.
%
object_pose(ClassInst, ObjectFrame, Pose) :-
    atom(ObjectFrame),
    py_call(ClassInst:object_pose(ObjectFrame), Pose).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% highlight_object(+ClassInstance, +IndivName, -HighLight)
%
%% Highlights an object.
%
highlight_object(ClassInst, IndivName, Highlight):-
    atom(IndivName),
    py_call(ClassInst:highlight(IndivName), Highlight).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% highlight_object_list(+ClassInstance, +IndivList, -Highlight)
%
%% Highlights a list of objects.
%
highlight_object_list(ClassInst, IndivList, Highlight):-
    py_call(ClassInst:highlight_list(IndivList), Highlight).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% highlight_indiv_list(+ClassInstance, +IndivList, -Highlight)
%
%% Highlights a list of individuals.
%
highlight_indiv_list(ClassInst, IndivList, Highlight):-
    py_call(ClassInst:highlight_indiv_list(IndivList), Highlight).
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% trajectory(+ClassInst, +IndivName, -Trajectory)
%
%% Shows a trajectory of the opening of an object.
%
trajectory(ClassInst, IndivName, Trajectory):-
    py_call(ClassInst:trajectory(IndivName), Trajectory).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% trajectory_list(+ClassInst, +IndivList, -Trajectory)
%
%% Shows trajectories of the opening of an object (list).
%
trajectory_list(ClassInst, IndivList, Trajectory):-
    py_call(ClassInst:trajectory_list(IndivList), Trajectory).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% highlight_trajectory(+ClassInst, +IndivList, -Highlight)
%
highlight_trajectory(ClassInst, IndivList, Highlight):-
    py_call(ClassInst:highlight_trajectory(IndivList), Highlight).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% open_door(+ClassInst, +ObjName, -Handle, -Highlight, -Trajectory)
%
%% Shows how a furniture item can be opended by 
%%     highlighting, trajectory of opening movement and object pose
%
open_door(ClassInst, ObjName, Info):-
    py_call(ClassInst:open_door(ObjName), Info).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% opening_information(+ClassInst, +ObjName, -Doors, -Handles)
%
%% Delivers informations on the doors and handles for opening a furniture item
%
opening_information(ClassInst, ObjName, Doors, Handles):-
    py_call(ClassInst:opening_information(ObjName), [Doors, Handles]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


