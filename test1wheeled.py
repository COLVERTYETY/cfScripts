    def callback(self, cf2):

	#cf2 is a : cf2=tf2_msgs.msg.TFMessage([t])
	#with t : t=geometry_msgs.msg.TransformStamped()

	#create cf to be a TransformStamped:
	cf = cf2.transforms[0]
	print(cf)

	x = cf.transform.translation.x
	y = cf.transform.translation.y
	z = cf.transform.translation.z
	print(z)


	if (z < 0.15):
	     z = 0.15

	for cf in allcfs.crazyflies:
		pos = np.array([x, y, z+0.5])
		print(pos)
		cf.cmdPosition(pos, 0)
                print(cf.position())
	#timeHelper.sleep(1.5+2.0)


    #print("press button to continue...")
    #swarm.input.waitUntilButtonPressed()


    def base_listener(self):
    	rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self.callback)
    	#timeHelper.sleep(1.5)


if __name__ == "__main__":
    rospy.init_node('follow_node', anonymous=True) # this is a maybe
