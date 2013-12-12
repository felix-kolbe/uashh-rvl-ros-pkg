
## own exports

from common_ros import ROSTopicCondition, ROSTopicAction

from introspection import Introspector

from runner import SMACHRunner


## used packages

import rgoap
import rospy


### set up rgoap-ros interface

## forward rgoap's logging to ROS
import logging

# ..for console output
import rosgraph.roslogging as _rl
logging.getLogger('rgoap').addHandler(_rl.RosStreamHandler())

# ..for network output (/rosout)
import rospy.impl.rosout as _ro
logging.getLogger('rgoap').addHandler(_ro.RosOutHandler())

# remove the default console output
rgoap.remove_default_loghandler()


## shutdown handling
rgoap.set_shutdown_check(rospy.is_shutdown)
