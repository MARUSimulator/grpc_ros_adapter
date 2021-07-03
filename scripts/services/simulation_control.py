from protobuf import simulation_control_pb2
from protobuf import simulation_control_pb2_grpc

import rospy

from rosgraph_msgs.msg import Clock


class SimulationControl(simulation_control_pb2_grpc.SimulationControlServicer):
    """
    Service used to control the flow of the simulation
    Simulator dictates the clock and does a Step request for ROS nodes
    """
    def __init__(self):
        self.clock_publisher = rospy.Publisher("clock", Clock, queue_size=10)
        self.startTime = -1
        self.currentTime = -1
        pass

    def SetStartTime(self, request, context):

        self.startTime = rospy.Time.from_sec(request.timeSecs)
        self.currentTime = self.startTime
        sim_clock = Clock()
        sim_clock.clock = self.startTime
        self.clock_publisher.publish(sim_clock)
        return simulation_control_pb2.SetStartTimeResponse(success=True)

    def Step(self, request, context):

        self.current_time = rospy.Time(secs=request.totalTimeSecs, nsecs=request.totalTimeNsecs)
        sim_clock = Clock()
        sim_clock.clock = self.currentTime
        self.clock_publisher.publish(sim_clock)
        return simulation_control_pb2.StepResponse(success=True)