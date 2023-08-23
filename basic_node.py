import rospy
from vrx_bridge_msgs.msg import ParamVec
from std_msgs.msg import Float64


class SimpleNode:
    def __init__(self):
        print("Starting node...")
        rospy.init_node('send_commands')
        self.rate = rospy.Rate(10)
        self.loopCount = 0

        self.taskInfo = 0
        self.taskType = ""
        self.taskState = ""

        # Send a thrust command
        self.thrustVal = 1.0

        # Basic subscribers
        self.taskSub = rospy.Subscriber('/vrx/task/info', ParamVec, self.taskCB)
 
        # Basic publishers
        self.rightThrustPub = rospy.Publisher('/wamv/thrusters/right/thrust', Float64, queue_size=10)
        self.leftThrustPub = rospy.Publisher('/wamv/thrusters/left/thrust', Float64, queue_size=10)


    def taskCB(self, _data):
        self.taskInfo = _data.params
        for p in self.taskInfo:
            if p.name == 'name':
                self.taskType = p.value.string_value
            if p.name == 'state':
                self.taskState = p.value.string_value

    def briefMoveForward(self):
        self.rightThrustPub.publish(self.thrustVal)
        self.leftThrustPub.publish(self.thrustVal)
        self.loopCount += 1

    def stop(self):
        self.rightThrustPub.publish(0.0)
        self.leftThrustPub.publish(0.0)

    def sendCmds(self):
        while not rospy.is_shutdown():
            try:  
                if self.taskType == "perception":
                    if self.taskState == ("initial" or "ready"):
                        print("Waiting for perception task to start...")
                    elif self.taskState == "running":
                        print("Perception running")
                    elif self.taskState == "finished":
                        print("Task ended...")
                        break
                elif self.taskType == "stationkeeping":
                    if self.taskState == ("initial" or "ready"):
                        print("Waiting for stationkeeping task to start...")
                    elif self.taskState == "running":
                        if self.loopCount < 10:
                            self.briefMoveForward()
                        else:
                            self.stop()
                    elif self.taskState == "finished":
                        print("Task ended...")
                        break
                else:
                    print(self.taskType)
                    if self.taskState == ("initial" or "ready"):
                        print("Waiting for default task to start...")
                    elif self.taskState == "running":
                        self.rightThrustPub.publish(self.thrustVal)
                        self.leftThrustPub.publish(self.thrustVal)
                    elif self.taskState == "finished":
                        print("Task ended...")
                        break

                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

sn = SimpleNode()
sn.sendCmds()
print('Complete')
