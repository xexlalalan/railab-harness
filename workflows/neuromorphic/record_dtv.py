#!/usr/bin/env python3
"""record_dtv.py <out.mp4>: write every /dtv/image frame to an mp4 until SIGINT/SIGTERM."""
import signal, sys, cv2, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

FPS = 2.0   # /dtv/image rate

def main():
    out = sys.argv[1]; rclpy.init(); n = Node("dtv_recorder"); w = [None]; count = [0]
    def cb(im):
        g = np.frombuffer(im.data, np.uint8).reshape(im.height, im.width)
        if w[0] is None: w[0] = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (im.width, im.height), False)
        w[0].write(g); count[0] += 1
    n.create_subscription(Image, "/dtv/image", cb, 2)
    stop = [False]
    for s in (signal.SIGINT, signal.SIGTERM): signal.signal(s, lambda *a: stop.__setitem__(0, True))
    while not stop[0]: rclpy.spin_once(n, timeout_sec=0.2)
    if w[0]: w[0].release()
    print("recorded %d frames -> %s" % (count[0], out))

if __name__ == "__main__": main()
