from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
import threading
import rospy
from robotiq_2f85_ros_pkg.srv import Move, MoveRequest

class Grasp(SkillDescription):

    def createDescription(self):
        pass

class grasp(PrimitiveBase):
    
    def createDescription(self):
        self.setDescription(Grasp(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):
        rospy.wait_for_service('/robotiq_2f85/move')
        service = rospy.ServiceProxy('/robotiq_2f85/move', Move)
        response = service(MoveRequest(0.0, 20.0, 60.0))
        self.result = response.error == 1
        self.done = True

    def onStart(self):
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True

    def execute(self):
        if not self.done:
            return self.step('Running...')

        self.thread.join()

        if self.result:
            return self.success('Done')
        else:
            return self.fail('Failed', -1)

class Release(SkillDescription):

    def createDescription(self):
        pass

class release(PrimitiveBase):
    
    def createDescription(self):
        self.setDescription(Release(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def run(self):
        rospy.wait_for_service('/robotiq_2f85/move')
        service = rospy.ServiceProxy('/robotiq_2f85/move', Move)
        response = service(MoveRequest(85.0, 20.0, 60.0))
        self.result = response.error == 0
        self.done = True

    def onStart(self):
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True

    def execute(self):
        if not self.done:
            return self.step('Running...')

        self.thread.join()

        if self.result:
            return self.success('Done')
        else:
            return self.fail('Failed', -1)
