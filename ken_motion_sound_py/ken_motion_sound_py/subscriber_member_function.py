# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import pyaudio
import wave
import time
import sys

chunk = 1024


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'ken_cmd_string',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        self.p = pyaudio.PyAudio()

    def __del__(self):
        self.p.terminate()

    def listener_callback(self, msg):
        dir = os.path.join(get_package_share_directory(
            'ken_driver'), 'sounds')
        wav = dir + "/" + msg.data + ".wav"
        self.get_logger().info('I heard: "%s"' % msg.data)

        try:
            wf = wave.open(wav, "rb")

            stream = self.p.open(format=self.p.get_format_from_width(wf.getsampwidth(
            )), channels=wf.getnchannels(), rate=wf.getframerate(), output=True)
            data = wf.readframes(chunk)

            stream.start_stream()
            while len(data) > 0:
                stream.write(data)
                data = wf.readframes(chunk)

            stream.stop_stream()
            stream.close()
            wf.close()
            self.get_logger().info('Finish sound: "%s"' % msg.data)
        except:
            print("error")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
