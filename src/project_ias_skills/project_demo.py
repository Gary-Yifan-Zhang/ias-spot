from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class ProjectDemo(SkillDescription):

    def createDescription(self):
        self.addParam('Arm', Element("rparts:ArmDevice"), ParamTypes.Required)
        self.addParam('PickLocation', Element('scalable:Workstation'), ParamTypes.Required)
        self.addParam('DriveLocation', Element('scalable:Workstation'), ParamTypes.Required)
        self.addParam('PlaceLocation', Element('scalable:Workstation'), ParamTypes.Required)

        self.addParam('Block', Element('skiros:TransformationPose'), ParamTypes.Required)
        self.addParam("BlockApproachPose", Element("skiros:TransformationPose"), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond("HasApproachPose","skiros:hasA", "Block", "BlockApproachPose", True))

        self.addParam('FreeSpace', Element('skiros:TransformationPose'), ParamTypes.Required)
        self.addParam("FreeSpaceApproachPose", Element("skiros:TransformationPose"), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond("HasApproachPose","skiros:hasA", "FreeSpace", "FreeSpaceApproachPose", True))

#################################################################################
# Implementations
#################################################################################

class project_demo(SkillBase):
    def createDescription(self):
        self.setDescription(ProjectDemo(), self.__class__.__name__)

    def expand(self, skill):
        # Change this to Serial
        skill.setProcessor(Sequential())
        skill(
            self.skill('SwitchController', '', specify={'Controller': 'joint_config'}),
            self.skill('ArmHome', 'arm_home'),
            self.skill('Drive', 'drive',
                       remap={'TargetLocation': 'PickLocation'}),
            self.skill('ArmLookout', 'arm_lookout'),
            self.skill('DetectBlock', 'detect_block',
                       remap={'Object': 'Block'}),
            self.skill('SwitchController', '', specify={'Controller': 'compliant'}),
            self.skill('LinearMovement', '',
                       remap={
                           'Target': 'BlockApproachPose'
                        }),
            self.skill('LinearMovement', '',
                       remap={
                           'Target': 'Block',
                        }),
            self.skill('Grasp', 'grasp'),
            self.skill('LinearMovement', '',
                       remap={
                           'Target': 'BlockApproachPose'
                        }),
            self.skill('SwitchController', '', specify={'Controller': 'joint_config'}),
            self.skill('ArmHome', 'arm_home'),
            self.skill('Drive', 'drive',
                       remap={'TargetLocation': 'DriveLocation'}),
            self.skill('Drive', 'drive',
                       remap={'TargetLocation': 'PlaceLocation'}),
            self.skill('ArmLookout', 'arm_lookout'),
            self.skill('DetectFreeSpace', 'detect_free_space',
                       remap={'PlaneLocation': 'FreeSpace'}),
            self.skill('SwitchController', '', specify={'Controller': 'compliant'}),
            self.skill('LinearMovement', '',
                       remap={
                           'Target': 'FreeSpaceApproachPose'
                        }),
            self.skill('LinearMovement', '',
                       remap={
                           'Target': 'FreeSpace',
                        }),
            self.skill('Release', 'release'),
            self.skill('LinearMovement', '',
                       remap={
                           'Target': 'FreeSpaceApproachPose'
                        }),
            self.skill('SwitchController', '', specify={'Controller': 'joint_config'}),
            self.skill('ArmHome', 'arm_home')
        )