import numpy as np
#Parameters
l1 = 0.3 #Link1 Length
l2 = 0.3 #Link2 Length
g = 9.81 #Acceleration due to gravity
h1 = 1.25 #Height of Joint1 starting from ground(y=0)
def release_point(D,h2,alpha):
    #D is the distance along the ground between target and Joint1
    #h2 is the height of the target from ground (y=0)
    #alpha is the slope of the target plane

    q1dbag = [] #q1dot list
    q2dbag = [] #q2dot list
    vxbag = [] #x velocity list
    vybag = [] #y velocity list
    q1list = [] #q1 list
    q2list = [] #q2 q2list
    q1bound = 46 #Upperbound of q1 iteration in degrees
    q2bound = 91 #Upperbound of q2 iteration in degrees
    for i in range(1,q1bound): #Iterating q1 from 1 to 45 degrees (Excluded 0 to avoid singularity of matrix)
      for j in range(1,q2bound): #Iterating q2 from 1 to 90 degrees (Excluded 0 to avoid singularity of matrix)
        q1 = np.pi*i/180 #degrees to radians
        q2 = np.pi*j/180 #degrees to radians
        xin = l1*np.sin(q1) + l2*np.sin(q1+q2) #Release point x-coordinate
        yin = h1 - (l1*np.cos(q1)+l2*np.cos(q1+q2)) #Release point y-coordinate
        vxin = ((0.5*g*(D-xin)**2)/((h2-yin)+(D-xin)*(1/ np.tan(np.deg2rad(alpha)))))**0.5 #x velocity calculation
        vyin = g*(D-xin)/vxin - vxin/np.tan(np.deg2rad(alpha)) #y velocity calculation
        vxbag.append(vxin)
        vybag.append(vyin)

        J = np.array([[l1*np.cos(q1)+l2*np.cos(q1+q2), l2*np.cos(q1+q2)],[l1*np.sin(q1)+l2*np.sin(q1+q2), l2*np.sin(q1+q2)]]) #Jacobian
        Jinv = np.linalg.inv(J) #Inverse of Jacobian
        qdots = np.dot(Jinv, np.array([[vxin],[vyin]])) #Joint angular velocities q1dot q2dot
        q1dbag.append(qdots[0,0]) #Collecting q1dots
        q2dbag.append(qdots[1,0]) #collecting q2dots
        q1list.append(i) #collecting q1's for convenience
        q2list.append(j) #collecting q2's for convenience

    #Finding the index where maximum of q1dot and q2dot is minimum
    maxdot = [max(abs(q1dbag[i]),abs(q2dbag[i])) for i in range((q1bound-1)*(q2bound-1))] #Finding maximum magnitude among q1dot and q2dot
    minmaxpos = maxdot.index(min(maxdot)) #Finding index where the maximum is minimum
    return np.array([q1list[minmaxpos],q2list[minmaxpos],q1dbag[minmaxpos],q2dbag[minmaxpos]]) #Returning q1,q2,q1dot,q2dot as an array


print(release_point(5, 0.3, 30))