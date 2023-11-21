import os
import time

import pytest
import launch
import launch.actions
from launch_ros.actions import Node
import launch_testing.actions
import rclpy
from rclpy import node
from rosidl_runtime_py import message_to_csv

from cpo_interfaces.msg import TDCP
from tests_common import TestRclpyTest

REF_FILE = 'tdcp_test_feb15c_REF.csv'
TEST_FILE = 'tdcp_test_feb15c.csv'


@pytest.mark.launch_test
def generate_test_description():
  dut = Node(
    package='cpo_frontend',
    executable='cpo_frontend',
    name='cpo_front',
    parameters=[{
      "from_serial": False,
      "log_serial": False,
      "data_path": "./../data/rtcm3/feb15c.BIN",
      "approximate_time": 1613400000,
      "use_sim_time": True,
      "enable_tropospheric_correction": True,
    }],
    additional_env={'PYTHONUNBUFFERED': '1'},
    output='screen'
  )

  clock_server = Node(
    package='cpo_frontend',
    executable='clock_server',
    name='clock_server',
    parameters=[{
      "first_meas_time": 1613419580,  # feb15c
      "playback_rate": 10,
    }],
    additional_env={'PYTHONUNBUFFERED': '1'},
    output='screen'
  )

  return launch.LaunchDescription([
    dut,
    clock_server,
    launch_testing.actions.ReadyToTest(),
  ])


class TdcpSubscriber(node.Node):
  """
  Subscribe to TDCP messages and save data to a CSV file so it can be used for regression testing.
  """

  def __init__(self):
    super().__init__('tdcp_test_subscriber')
    self.tdcp_sub = self.create_subscription(
      TDCP,
      'tdcp',
      self.tdcp_callback,
      10
    )
    self.tdcp_sub  # prevent unused variable warning
    self.tdcp_msg_count = 0

  def tdcp_callback(self, msg: TDCP):
    """Subscribes to TDCP msgs from frontend and saves to CSV."""
    self.tdcp_msg_count += 1

    with open('tdcp_test_feb15c.csv', 'a') as file:
      file.write(message_to_csv(msg) + '\n')


class TestTdcpRegression(TestRclpyTest):

  def setUp(self):
    if os.path.exists(TEST_FILE):
      os.remove(TEST_FILE)

    # Setup and let spin
    self.node = TdcpSubscriber()
    end_time = time.time() + 60.0
    while time.time() < end_time:
      rclpy.spin_once(self.node, timeout_sec=0.1)

  def tearDown(self):
    self.node.destroy_node()

  def test_tdcp_output(self, proc_output):
    """
    Test front-end output given the same data, has not changed.
    """

    # check we had output
    assert self.node.tdcp_msg_count > 20

    # compare output file to reference file
    f_ref = open(REF_FILE, 'r')
    f_test = open(TEST_FILE, 'r')

    for ref_line in f_ref:
      assert ref_line == f_test.readline()
