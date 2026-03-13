
#HOW RUN : -> Scrivania/backendProvas/sdk$ PYTHONPATH=src:$PYTHONPATH python3 src/cfswarm_sdk/examples/first_ex_opti.py



from cfswarm_sdk.context import RosContext
from cfswarm_sdk.optitrack import OptiTrack, OptiTrackConfig
import time

context  = RosContext()

opti_cfg = OptiTrackConfig(
    pose_topics={
        "tb1": "/mocap/tb1/pose",
        "tb3": "/mocap/tb3/pose",
    }
)


context.start()



o = OptiTrack(context.node, opti_cfg)

time.sleep(2)  # wait for the first messages to arrive

print("Latest pose for tb1:", o.get_pose("tb1"))
print("Snapshot of all poses:", o.snapshot())
print("Latest pose for tb3:", o.get_pose("tb3"))
print("Snapshot of all poses:", o.snapshot())


context.shutdown()