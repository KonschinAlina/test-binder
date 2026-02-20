import ipywidgets as widgets
from IPython.display import display, clear_output
from owlready2 import *
import math

import sys
import rospy

from knowrob_ros.knowrob_ros_lib import (
    KnowRobRosLib,
    TripleQueryBuilder,
    graph_answer_to_dict,
    graph_answers_to_list,
    get_default_modalframe,
)
from knowrob_ros.msg import (
    AskAllResult,
    AskIncrementalResult,
    AskIncrementalNextSolutionResult,
    AskOneResult,
    TellResult,
)

from owlready2 import *

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA

sys.path.append("/home/jovyan/work/src/lib")
import janus_swi as janus
janus.consult("/home/jovyan/work/prolog/funcs.pl")

def start_UI(t, janus):

    # Check whether map is loaded
    # if not t.map_server_running():
    #     print(f"No semantic map loaded!")
    #     return

    onto = get_ontology('/home/jovyan/work/prolog/BA-class_extraction1.owl').load()
 
    # --- 1. Setup and Container ---
    output_info = widgets.Output()
    dynamic_area = widgets.VBox()
    
    # Get matching of links to classes
    # matchings: [{"link": "...", "class": "..."}, ...]
    matchings = t.link_class_matching()

    if matchings is None:
        print("No link-class matchings found!")
    else:
        data = []
        # Save matching as list
        for item in matchings:
            data.append({"link": item["link"], "class": item["class"].capitalize()})
    
        # Get all occuring classes in urdf, no duplicates using set 
        available_classes = sorted(list(set(entry["class"] for entry in data)))
        
        # Widget functions for hierarchie (hasPart)
        def part_widget(indiv_name, recursive=False, is_kinematic=False):
            # create button for current indiv
            btn = widgets.Button(description=indiv_name, layout=widgets.Layout(width='auto'))

            # Get individual object to indiv_name
            ind = onto.search_one(iri=f"*{indiv_name}")
            
            # Get all parts of individual (hasPart) 
            parts = []
            if ind and hasattr(ind, "hasPart"):
                parts = [p.name for p in ind.hasPart]
            
            # Create button
            btn = widgets.Button(
                description=f"{indiv_name}", 
                layout=widgets.Layout(width='auto'),
                button_style = 'primary' if is_kinematic else ''
            )

            # Callback for clicking on a button
            # Calling highlight 
            def on_click(b):
                with output_info:
                    clear_output()
                    print(f"Highlighting: {indiv_name}")
                    janus.query_once("highlight_object(T, Obj, Highlight)", {'T': t, 'Obj': indiv_name})
            btn.on_click(on_click)
            
            # Check, if recursive search is needed for parts
            if recursive:
                if parts:
                    child_widgets = [part_widget(p, recursive=True) for p in parts]
                else:
                    child_widgets = [widgets.HTML(f"<i style='color:gray; padding-left:20px;'>No further parts for {indiv_name}</i>")]
                
                # Create container for part elements
                child_box = widgets.VBox(child_widgets, layout=widgets.Layout(padding='0 0 0 20px'))
                # Create foldable accordion
                accordion = widgets.Accordion(children=[child_box])
                accordion.set_title(0, f"Parts of: {indiv_name}")
                # Set accordion to closed
                accordion.selected_index = None
    
                return widgets.VBox([btn, accordion])
            
            return btn
            
        # --- 1. KINEMATIC CHAIN ---
        def show_kinematic_ui():
            dropdown = widgets.Dropdown(options=available_classes, description='Class:')
            tree_box = widgets.VBox()
        
            def on_root_change(change):
                selected_class = change['new']
                
                # Get all indivs of selected class
                indivs = [d["link"] for d in data if d["class"] == selected_class]
                
                # Build accordions for indivs using part_widget
                tree_box.children = [part_widget(ind, recursive=True, is_kinematic=True) for ind in indivs]
        
            dropdown.observe(on_root_change, names='value')
            on_root_change({'new': dropdown.value})
            
            return widgets.VBox([
                widgets.HTML("<b>Kinematic Hierarchy:</b>"),
                dropdown, 
                tree_box
            ])
        
        
        # --- 2. CLASSES ---
        def show_classes_ui():
            accordion_box = widgets.VBox()
            dropdown = widgets.Dropdown(options=available_classes, description='Class:')

            def on_class_change(change):
                selected_class = change['new']
                # Get all indivs of selected class
                indivs = [d["link"] for d in data if d["class"] == selected_class]
                
                # Create an 'All' button which calls highlight_object_list
                all_btn = widgets.Button(description=f"All {selected_class}s", button_style='success')
                
                def on_all_click(b):
                    with output_info:
                        clear_output()
                        janus.query_once("highlight_object_list(T, Class, Highlight)", {'T': t, 'Class': selected_class.lower()})
                all_btn.on_click(on_all_click)

                # Create accordion for all button and parts
                accordion_box.children = [all_btn] + [part_widget(ind) for ind in indivs]
        
            dropdown.observe(on_class_change, names='value')
            on_class_change({'new': dropdown.value})
            return widgets.VBox([dropdown, accordion_box])

        
        # --- 3. TRAJECTORIES ---
        def show_trajectories_ui():
 
            # Get all joint types and their names (which exist in urdf) from database
            # no class duplicates using set
            joint_class = onto.Joint
            joint_subclasses = list(set(joint_class.subclasses()))
            subclasses_names = [j.name for j in joint_subclasses]

            # Get fixed joint classs
            fixed_joint_class = onto.Fixed_Joint

            # Get all joint individuals and their names
            all_joint_indivs = joint_class.instances()
            joint_indiv_names = [j.name for j in all_joint_indivs]
        
            # Extract only joints that are not fixed
            moving_joints = [j for j in joint_subclasses if j != fixed_joint_class]
           
            joint_box = widgets.VBox()
            type_dropdown = widgets.Dropdown(options=subclasses_names, description='Joint Type:')
            
        
            def on_type_change(change):
                selected_type = change['new']
                target_type = onto[selected_type]

                # Check, if target type can be determined (exists in ontology)
                if target_type is None:
                    with output_info:
                        print(f"selected_type: {selected_type} not found")
                    return
                
                # Get all joints of a certain type
                matching_joints = [j for j in all_joint_indivs if isinstance(j, target_type)]
        
                buttons = []
                
                # Get the child link of the choosen joint for trajectory
                current_child_link = None 
                for joint in matching_joints:
                    if hasattr(joint, "hasChildLink") and joint.hasChildLink:
                        current_child_name = joint.hasChildLink[0]
                        print(f"current_child_name: {current_child_name}")
                    else:
                        print(f"joint: {joint.name} has no child link!")
                        print(f"Using joint name as fallback.")
                        #current_child_name = joint.name
                        current_child_name = joint
                      
                    btn = widgets.Button(description=f"{joint.name}", layout=widgets.Layout(width='auto'))
                    
                    def on_joint_click(b, link=current_child_name):
                        with output_info:
                            clear_output()
                            janus.query_once("trajectory(T, Childlink, Trajectory)", {'T': t, 'Childlink': link.name}) 
                    
                    btn.on_click(on_joint_click)
                    buttons.append(btn)
                
                joint_box.children = buttons
        
            type_dropdown.observe(on_type_change, names='value')
            on_type_change({'new': type_dropdown.value})
            return widgets.VBox([type_dropdown, joint_box])
        
        
        # --- 5. Main Drop Down ---
        mode_dropdown = widgets.Dropdown(
            options=['Classes', 'Trajectories', 'Kinematic Chain'],
            value='Classes',
            description='Mode:',
            style={'description_width': 'initial'}
        )
        
        def on_mode_change(change):
            dynamic_area.children = []
            with output_info: 
                clear_output()
                
            if change['new'] == 'Classes':
                dynamic_area.children = [show_classes_ui()]
            elif change['new'] == 'Trajectories':
                dynamic_area.children = [show_trajectories_ui()]
            else:
                dynamic_area.children = [show_kinematic_ui()]
        
        mode_dropdown.observe(on_mode_change, names='value')
    
        # Initalization, start with Classes in drop down menu
        display(mode_dropdown, dynamic_area, output_info)
        on_mode_change({'new': 'Classes'})
    