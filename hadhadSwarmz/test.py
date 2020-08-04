def bab(msg):
	
	global x
	x = msg.data
	eb3t()

def bab1(msg):
	
	global x1
	x1 = msg.data
	eb3t()

def bab2(msg):
	
	global x2
	x2 = msg.data
	eb3t()


	global status
	counter = 0
	global x
	global x1
	global x2
	#print(counter)
	if x == True:
	   	counter = 1
           	status  = 2
	if(x1 == True and counter == 1 ):
	   counter = 2
	if(x2 == True and counter == 2 ):
	   counter = 3
	if(x2 == False and counter == 3 ):
           status = 1
	   counter= 0

	subs_bab[0] = rospy.Subscriber('/rob1/bab',Bool,bab)
	subs_bab[1] = rospy.Subscriber('/rob2/bab',Bool,bab1)
	subs_bab[2] = rospy.Subscriber('/rob3/bab',Bool,bab2)


