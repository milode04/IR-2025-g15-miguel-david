import random
for i in range (20):
    a = random.randint (2, 200)
    b = random.randint (2, 200)
    if (a<40 and b<40):
        print ("stop")
    elif (a<40 and b>40):
        print ("turn right")
    elif (a>40 and b<40):
        print ("turn left")
    else:
        print ("forward")