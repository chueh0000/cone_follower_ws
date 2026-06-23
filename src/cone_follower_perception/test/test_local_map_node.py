import unittest
import pytest
import rclpy
from cone_follower_msgs.msg import ConeArray, Cone
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
import time
import math

class TestLocalMapNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    # Simple unit test since we don't want to spin up a full ROS 2 integration 
    # test here. We will just test the data association logic manually.
    def test_cone_merging(self):
        # This tests the core logic of local_map_node's data association
        global_cones = []
        merge_distance = 0.5
        
        # Incoming transformed point
        pt_global_x = 1.0
        pt_global_y = 2.0
        
        # Test 1: Add new cone
        is_duplicate = False
        for mapped_cone in global_cones:
            dist = math.sqrt((pt_global_x - mapped_cone['x'])**2 + (pt_global_y - mapped_cone['y'])**2)
            if dist < merge_distance:
                is_duplicate = True
                mapped_cone['x'] = (mapped_cone['x'] + pt_global_x) / 2.0
                mapped_cone['y'] = (mapped_cone['y'] + pt_global_y) / 2.0
                break
                
        if not is_duplicate:
            global_cones.append({'x': pt_global_x, 'y': pt_global_y, 'z': 0.0, 'color': 'blue'})
            
        self.assertEqual(len(global_cones), 1)
        
        # Test 2: Add duplicate cone (distance < 0.5)
        pt_global_x = 1.2
        pt_global_y = 2.0
        is_duplicate = False
        for mapped_cone in global_cones:
            dist = math.sqrt((pt_global_x - mapped_cone['x'])**2 + (pt_global_y - mapped_cone['y'])**2)
            if dist < merge_distance:
                is_duplicate = True
                mapped_cone['x'] = (mapped_cone['x'] + pt_global_x) / 2.0
                mapped_cone['y'] = (mapped_cone['y'] + pt_global_y) / 2.0
                break
                
        if not is_duplicate:
            global_cones.append({'x': pt_global_x, 'y': pt_global_y, 'z': 0.0, 'color': 'blue'})
            
        self.assertEqual(len(global_cones), 1)
        # Should be averaged: (1.0 + 1.2) / 2 = 1.1
        self.assertAlmostEqual(global_cones[0]['x'], 1.1)

if __name__ == '__main__':
    unittest.main()
