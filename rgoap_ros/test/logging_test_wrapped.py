'''
Created on Dec 11, 2013

@author: felix
'''

import logging

import rgoap_ros # this reconfigures rgoap's logging

import rospy


def test():
    rospy.init_node('logging_test_remote')

    rospy.sleep(2)

    logger = logging.getLogger('rgoap')

    logger.error("msg..logging.error (%s)", "arg")
    logger.info("msg..logging.info (%s)", "arg")




if __name__ == '__main__':
    test()
#    rospy.spin()
