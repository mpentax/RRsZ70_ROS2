"""Latest-only typed frame/metadata join, shared by ROS observation consumers.

Images remain visible when uncalibrated, but metric consumers never process them.
No HTTP, camera control or acquisition is performed here.
"""
import time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, CompressedImage
from z70_interfaces.msg import FrameMetadata


class MatchedFrames:
    def __init__(self, node, camera, callback, compressed=False, on_session=None):
        self.callback=callback; self.on_session=on_session
        self.frame=None; self.metadata=None; self.used=None; self.session=None
        self.group=MutuallyExclusiveCallbackGroup()
        topic=f'/camera/{camera}'
        self.subscriptions=[
            node.create_subscription(CompressedImage if compressed else Image,
                topic+('/image_raw/compressed' if compressed else '/image_raw'), self.image, 2, callback_group=self.group),
            node.create_subscription(FrameMetadata,topic+'/metadata',self.info,2,callback_group=self.group)]

    @staticmethod
    def key(msg):
        return (msg.header.stamp.sec,msg.header.stamp.nanosec,msg.header.frame_id)

    def image(self, msg):
        self.frame=msg;self.match()

    def info(self, msg):
        self.metadata=msg;self.match()

    def match(self):
        frame,meta=self.frame,self.metadata
        if frame is None or meta is None or self.key(frame)!=self.key(meta):return
        e=meta.evidence
        ident=(bytes(e.session_id.uuid),e.sequence)
        if ident==self.used:return
        self.used=ident
        age=time.time()-(e.received_at.sec+e.received_at.nanosec/1e9)
        if not e.valid or e.stale or not meta.calibration_id or age<0 or age>5.:return
        if self.session is not None and self.session!=ident[0] and self.on_session:self.on_session()
        self.session=ident[0]
        self.callback(frame)
