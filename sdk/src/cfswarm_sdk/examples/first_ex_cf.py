#HOW RUN : -> Scrivania/backendProvas/sdk $PYTHONPATH=src:$PYTHONPATH python3 src/cfswarm_sdk/examples/first_ex_cf.py



from cfswarm_sdk.optitrack import OptiTrack, OptiTrackConfig
from cfswarm_sdk.context import RosContext
from cfswarm_sdk.crazyflie_agent import CrazyflieAgent, AgentConfig
from cfswarm_sdk.swarm import Swarm, SwarmConfig
import time



context = RosContext()


#---CONFIGURATONs--
agent_cfg1 = AgentConfig(
    drone_id="cf1",
    cmd_pos_topic="/cf1/cmd_pos",
    cmd_pos_rel_topic="/cf1/cmd_pos_relative",
    takeoff_srv="/cf1/takeoff",
    land_srv="/cf1/land",
    ekf_reset_srv="/cf1/ekf_reset"
)

agent_cfg2 = AgentConfig(
    drone_id="cf2",
    cmd_pos_topic="/cf2/cmd_pos",
    cmd_pos_rel_topic="/cf2/cmd_pos_relative",
    takeoff_srv="/cf2/takeoff",
    land_srv="/cf2/land",
    ekf_reset_srv="/cf2/ekf_reset" 
)


swarm_cfg = SwarmConfig(
    mocap_pose_topics={
        "cf1": "/mocap/cf1/pose",
        "cf2": "/mocap/cf2/pose",
    },
    agents={
        "cf1": agent_cfg1,
        "cf2": agent_cfg2,
    }
)


#---CONFIGURATONs--


context.start()

#example without swarm, just two agents and no optitrack (so no pose feedback, just to test the connection and topics)
#cf1 = CrazyflieAgent(context.node, agent_cfg1)
cf2 = CrazyflieAgent(context.node, agent_cfg2)

#try using swarm with the same two agents and no optitrack (so no pose feedback, just to test the connection and topics)
#opti= OptiTrack(context.node, OptiTrackConfig(pose_topics=swarm_cfg.mocap_pose_topics))
#swarm = Swarm(context.node, swarm_cfg)


time.sleep(3)
#swarm.agent("cf1").go_to_abs(0.0, 0.0, 0.1)
print("Agent ready:" , cf2)

cf2.go_to_abs(0.0, 0.0, 0.1)
#time.sleep(2)
#cf2.set_vel_body(vx=1.0, vy=0.0, vz=0.1, yaw_rate=0.0)
time.sleep(2)
#cf2.stop()
#print("Agent ready:" , swarm.agent("cf1"))
#print("Agent ready:" , swarm.agent("cf2"))
context.shutdown()