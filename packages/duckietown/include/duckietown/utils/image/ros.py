from PIL import Image
import rospy
import numpy as np

from sensor_msgs.msg import Image, CompressedImage


def rgb_to_pil(im: np.ndarray):
    return Image.fromarray(im)


def rgb_to_imgmsg(im: np.ndarray) -> CompressedImage:
    im = rgb_to_pil(im)
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.height = im.height
    msg.width = im.width
    msg.encoding = "rgb8"
    msg.is_bigendian = False
    msg.step = 3 * im.width
    msg.data = np.array(im).tobytes()
    return msg


def rgb_to_compressed_imgmsg(im) -> CompressedImage:
    im = im.convert('RGB')
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = image.tobytes()
    msg.data = np.array(im).tobytes()
    return msg