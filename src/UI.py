import ipywidgets as widgets
from IPython.display import display, clear_output
from owlready2 import *
import math

import sys
import rospy

# ANSI color codes
GREEN = '\033[92m'
RED = '\033[91m'
RESET = '\033[0m'

from owlready2 import *

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header, ColorRGBA

sys.path.append("/opt/ros/overlay_ws/src/project/src/lib")
import janus_swi as janus
janus.consult("/opt/ros/overlay_ws/src/project/prolog/funcs.pl")

###################################################################
## Start UI
#
# This function creates and starts the UI in the notebook
#
# Input: janus instance
# Output: starts the UI
#
#
def start_UI(t, janus):

    # Checks whether the map is loaded
    if not t.param_server_running():
        rospy.loginfo(RED + f"No semantic map loaded!" + RESET)
        return

    onto = get_ontology('/opt/ros/overlay_ws/src/project/prolog/BA-class_extraction-SOMA.owl').load()

    # --- 1. Setup and Container ---
    output_info = widgets.Output()
    dynamic_area = widgets.VBox()
    
    data = []
    try:
        link_class = onto.search_one(iri="*Link")
        #print(link_class)
        if link_class:
            for link_inst in link_class.instances():
                classes = [c for c in link_inst.is_a if hasattr(c, "iri") and c.name != "Link"]
                
                if classes:
                    target_class = classes[0]
                    data.append({
                        "link": link_inst.name,
                        "class" : target_class.iri
                    })
    except Exception as e:
        print(f"Error 1 {e}")
        
    if not data:
        print(f"Error 2")
        return
        
    else:
        # Retrieves all occuring classes in the URDF file, no duplicates using set 
        class_dict = {
            entry["class"].split('#')[-1]: entry["class"] for entry in data}
        
        available_classes = sorted(class_dict.items())
        #print(f"available_classes: {available_classes}")
        #available_classes = sorted(list(set(entry["class"] for entry in data)))
        
        # Widget functions for hierarchie (hasPart)
        def part_widget(indiv_name, recursive=False, is_kinematic=False):
            
            # Creates a button for the current individual
            btn = widgets.Button(description=indiv_name, layout=widgets.Layout(width='auto'))

            # Retrieves the individual object associated with the indiv_name
            ind = onto.search_one(iri=f"*{indiv_name}")
            
            # Retrieves all parts of an individual (hasPart) 
            parts = []
            if ind and hasattr(ind, "hasPart"):
                parts = [p.name for p in ind.hasPart]
            
            # Creates button
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
                    print(GREEN + f"Highlighting:" + RESET + f"'{indiv_name}'")
                    res = janus.query_once("highlight_object(T, Obj, Highlight)", {'T': t, 'Obj': indiv_name})
                    
                    if res and res.get('Highlight') != None:
                        print(res)
                        
            btn.on_click(on_click)
            
            # Checks, if recursive search is needed for parts
            if recursive:
                if parts:
                    child_widgets = [part_widget(p, recursive=True, is_kinematic=is_kinematic) for p in parts]
                else:
                    child_widgets = [widgets.HTML(f"<i style='color:gray; padding-left:20px;'>No further parts for {indiv_name}</i>")]
                
                # Creates a container for the part elements
                child_box = widgets.VBox(child_widgets, layout=widgets.Layout(padding='0 0 0 20px'))
                
                # Creates a foldable accordion, sets it to closed
                accordion = widgets.Accordion(children=[child_box])
                accordion.set_title(0, f"Parts of: {indiv_name}")
                accordion.selected_index = None
    
                return widgets.VBox([btn, accordion])
            
            return btn
            
        # --- 1. KINEMATIC CHAIN ---
        # Created the UI for the mode: kinematic chain
        def show_kinematic_ui():
            dropdown = widgets.Dropdown(options=available_classes, description='Class:')
            tree_box = widgets.VBox()
        
            def on_root_change(change):
                selected_class_iri = change['new']
                
                # Retrieves all individuals of the selected class
                indivs = [d["link"] for d in data if d["class"] == selected_class_iri]
                
                # Builds an accordions for the individuals using part_widget
                tree_box.children = [part_widget(ind, recursive=True, is_kinematic=True) for ind in indivs]
        
            dropdown.observe(on_root_change, names='value')
            on_root_change({'new': dropdown.value})
            
            return widgets.VBox([
                widgets.HTML("<b>Kinematic Hierarchy:</b>"),
                dropdown, 
                tree_box
            ])
        
        
        # --- 2. CLASSES ---
        # Created the UI for the mode: classes
        def show_classes_ui():
            # Initializes an accordion box
            accordion_box = widgets.VBox()
            dropdown = widgets.Dropdown(options=available_classes, description='Class:')

            def on_class_change(change):
                selected_class_iri = change['new']
                #print(f"selected_class_iri:{selected_class_iri}")
                
                short_name = selected_class_iri.split('#')[-1].lower()
                #print(f"short_name:{short_name}")
                
                # Retrieves all individuals of the selected class
                indivs = [d["link"] for d in data if d["class"] == selected_class_iri]
                
                # Creates an 'All' button which calls highlight_object_list
                all_btn = widgets.Button(description=f"All {short_name}s", button_style='success')
                
                def on_all_click(b):
                    with output_info:
                        clear_output()
                        res = janus.query_once("highlight_object_list(T, Class, Highlight)", {'T': t, 'Class': selected_class_iri})
                        if res and res.get('Highlight') != None:
                            print(res)
                        else:
                            rospy.loginfo(RED + f"Individuals have no visual links" + RESET)
                            
                all_btn.on_click(on_all_click)

                # Creates an accordion for all button and parts
                accordion_box.children = [all_btn] + [part_widget(ind) for ind in indivs]
        
            dropdown.observe(on_class_change, names='value')
            on_class_change({'new': dropdown.value})
            return widgets.VBox([dropdown, accordion_box])

        
        # --- 3. TRAJECTORIES ---
        # Created the UI for the mode: trajectories
        def show_trajectories_ui():
 
            # Retrieves all joint types and their names from the ontology
            # no class duplicates using set
            namespace_joint = onto.get_namespace("http://www.ease-crc.org/ont/SOMA.owl#")
            joint_class = namespace_joint.Joint
            joint_subclasses = list(set(joint_class.subclasses()))
            subclasses_names = [j.name for j in joint_subclasses]

            # Retrieve the fixed joint class
            fixed_joint_class = namespace_joint.FixedJoint

            # Retrieves all joint individuals and their names
            all_joint_indivs = joint_class.instances()
            joint_indiv_names = [j.name for j in all_joint_indivs]
        
            # Extracts only joints that are not fixed
            moving_joints = [j for j in joint_subclasses if j != fixed_joint_class]
           
            joint_box = widgets.VBox()
            type_dropdown = widgets.Dropdown(options=subclasses_names, description='Joint Type:')
            
        
            def on_type_change(change):
                selected_type = change['new']
                target_type = namespace_joint[selected_type]

                # Checks if the target type can be determined (exists in ontology)
                if target_type is None:
                    with output_info:
                        rospy.loginfo(RED + f"selected_type: {selected_type} not found" + RESET)
                    return
                
                # Retrieves all joints of a certain type
                matching_joints = [j for j in all_joint_indivs if isinstance(j, target_type)]
        
                buttons = []
                
                # Retrieves the child link of the choosen joint for trajectory
                current_child_link = None 
                for joint in matching_joints:
                    if hasattr(joint, "hasChildLink") and joint.hasChildLink:
                        current_child_name = joint.hasChildLink[0]
                        print(f"current_child_name: {current_child_name}")
                    else:
                        print(f"joint: {joint.name} has no child link!")
                        #continue
                        current_child_name = joint
                      
                    btn = widgets.Button(description=f"{joint.name}", layout=widgets.Layout(width='auto'))
                    
                    def on_joint_click(b, link=current_child_name):
                        with output_info:
                            clear_output()
                            res = janus.query_once("trajectory(T, Childlink, Trajectory)", {'T': t, 'Childlink': link.name}) 
                            print(res)
                    
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
    
