#!/usr/bin/env python3

from __future__ import print_function

from nlp_pkg.srv import AddTwoInts, AddTwoIntsResponse
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import SetBool, SetBoolResponse
import rospy

# def handle_add_two_ints(req):
#     print("Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b)))
#     return AddTwoIntsResponse(req.a + req.b)

# def add_two_ints_server():
#     rospy.init_node('add_two_ints_server')
#     s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
#     print("Ready to add two ints.")
#     rospy.spin()


def serverOn():
    rospy.init_node('test_server_node')
    s = rospy.Service('call_service', SetBool, handle_call)
    print("Server is on!")
    rospy.spin()


def handle_call(req):
    print("call server by " + str(req.data))
    return SetBoolResponse(True, str(req.data))


if __name__ == "__main__":
    # add_two_ints_server()
    serverOn()
