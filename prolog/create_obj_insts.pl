%% This module loads and creates objects from the semantic map in the database.

:- module(furniture_creation,
	  [
	      load_urdf_from_param(+),
	      init_furnitures/0
	  ]).

% 1. name of instance
% 2. knowledge role
% 3. link name 
% = iai_kitchen/couch:couch:place_one

% check for suffix
is_semantic_map_object(Link):-
    (sub_string(Link, _,_,_,"place_one")),
    !.

urdf_link_class(UrdfLink, Class, KnowledgeRole) :-
    atomic_list_concat([_,KnowledgeRole,_], ':', UrdfLink),
    link_role_class(KnowledgeRole, Class),
    !.

    
% link obj to a class
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"couch"),
    Class = knowrob:'Couch',
    !.


% UrdfLink = couch:couch:place_one
furniture_pose(UrdfLink, [ObjectFrame, [0,0,0], [0,0,0,1]]) :-
    atom_concat('iai_kitchen/', UrdfLink, ObjectFrame).


furniture_shape(UrdfLink, ShapeTerm) :-
    get_urdf_id(URDF),
    collision_link(UrdfLink, CollisionLink),
    urdf_link_collision_shape(URDF, CollisionLink, ShapeTerm, _).

link_role_class(couch, knowrob:'Couch') :- !.

init_furniture(UrdfLink) :-
    sub_string(UrdfLink, _, _, _, "couch"),
    ros_info("Custom init for couch: ~w", [UrdfLink]),
    urdf_link_class(UrdfLink, ClassTerm, RobocupName),
    rdf_global_id(ClassTerm, Class),
    furniture_pose(UrdfLink, Pose),
    furniture_shape(UrdfLink, ShapeTerm),
    create_object(Furniture, Class, Pose, [shape(ShapeTerm), data_source(semantic_map)]),
    kb_project((
        has_urdf_name(Furniture, UrdfLink),
        has_robocup_name(Furniture, RobocupName)
    )),
    !.






