import unittest

import rclpy


class TestRclpyTest(unittest.TestCase):
  @classmethod
  def setUpClass(self):
    # Initialize the ROS context for the test node
    rclpy.init()

  @classmethod
  def tearDownClass(self):
    # Shutdown the ROS context
    rclpy.shutdown()

  def setUp(self):
    pass

  def tearDown(self):
    pass
