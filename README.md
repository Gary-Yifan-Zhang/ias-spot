<!--
 * @Author: DexterZzz1010 DexterZzz1010@gmail.com
 * @Date: 2024-10-03 15:41:56
 * @LastEditors: DexterZzz1010 DexterZzz1010@gmail.com
 * @LastEditTime: 2024-10-07 20:05:47
 * @FilePath: /catkin_ws/src/Ias-Skills-Handout/README.md
 * @Description: vivo50
 * 
 * Copyright (c) 2024 by DexterZzz1010@gmail.com, All Rights Reserved. 
-->
# ias-skills-handout

Skeleton repository for Skiros skills

=======
## SSH
SSH key in order to work on the lab computer
```
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIMfxSKdXYtJ2fqd+gcPJY4yMWTp1JBG3czZF1iZz653F ias_2024_spot@robot
```

## Using project_ias_skills

At very begainning, after pulling the repo and compiling, we found that there were two `ias_skills`, located at `your_repo_name/src/ias_skills` and `catkin_ws/src/libs/skiros2/ias_skills`. This caused the workspace to fail the catkin build, so I renamed the folder in the repo directory into`project_ias_skills`.

However, this action poses a risk and may prevent us from accessing functions within the `ias_skills` folder under the repo path, and Skiros will not be able to use the executable files.

The following steps will allow 'project_ias_skills' to be imported and called properly:

1. **VERY IMPORTANT：** Modified `catkin_ws/src/start/heron_launch/launch/simulation.launch` **line 18**  into 
```
    <arg name="libraries_list" value="[skiros2_std_skills, skiros2_moveit_lib, project_ias_skills]"/>
```

2. Modified `catkin_ws/src/your_repo_name/src/package.xml` **line 3**  into 
```
  <name>project_ias_skills</name>
```

3. Modified `catkin_ws/src/your_repo_name/src/setup.py` **line 7**  into 
```
 setup_args = generate_distutils_setup(
    packages=['project_ias_skills'],
    package_dir={'': 'src'},
)
```

4. Modified `catkin_ws/src/your_repo_name/src/CMakeLists.txt` **line 2**  into 
```
 project(project_ias_skills)
```

5. Changed the import source in the action_server.py
```
from project_ias_skills.perception_utilities import *
from project_ias_skills.transform_utils import orthonormalize_rotation
```

6. Run the following commands in Docker:
```
catkin build
```

## Navigation

To update the default navigation goal in SkiROS:

Edit the following line in the file located at: `catkin_ws/src/start/heron_launch/owl/scenes/heron.turtle`, **line 55**:

```ttl
scalable:Workstation-1 a scalable:Workstation,
        owl:NamedIndividual ;
    rdfs:label "LocationPickupTable" ;
    skiros:BaseFrameId "map"^^xsd:string ;
    skiros:DiscreteReasoner "AauSpatialReasoner"^^xsd:string ;
    skiros:FrameId "scalable:Workstation-1"^^xsd:string ;
    skiros:OrientationW "-0.706312726538"^^xsd:float ;
    skiros:OrientationX "0.0"^^xsd:float ;
    skiros:OrientationY "0.0"^^xsd:float ;
    skiros:OrientationZ "0.707899945141"^^xsd:float ;
    skiros:PositionX "42.7"^^xsd:float ;
    skiros:PositionY "3.3"^^xsd:float ;
    skiros:PositionZ "0.0"^^xsd:float ;
    skiros:PublishTf true ;
    skiros:Template "scalable:workstation_template"^^xsd:string ;
    skiros:hasTemplate scalable:workstation_template .
```

## Perception
To add detect_button skill into SkiROS:
Modified the following line in `/ros-container/home/catkin_ws/src/start/heron_launch/launch/simulation.launch`, **line 20**:

```
  <arg name="skill_list" value="[detect_button,drive, arm_lookout, arm_home, grasp, release, switch_controller, go_to_linear, project_demo]"/>
```

To add button_perception_action_server.py into launch flie:
Modified the following line in `/ros-container/home/catkin_ws/src/start/heron_launch/launch/general.launch`, **line 64**:
```
  <node pkg="project_ias_skills" name="button_perception_action_server" type="button_perception_action_server.py" output="screen">

  </node>
```


## Behaviour tree
![Behaviour tree not found](./docs/Behavior%20tree%20V1.png "Behaviour tree")
### Skill descrpitions
* `navigate_to_button`
Navigate to hard-coded position. Fails until final position is reached.
* `find_button`
Looks for button in camera frame. Fails as long as the there is no found button.
* `align_button`
Moves the robot arm to an position aligned with the button and its normal. Fails as long as the endeffector is not aligned with button.
* `push_button`
Combines force feedback and arm movement to press the button. Fails until the force feedback is adequate.

### Assumptions
* Button is always found upon lookout
* End effector is normal to the button in the lookout position
* We can always get a pose estimate of the button
* Navigation always places us close enough to the button

