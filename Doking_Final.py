import rospy
import math
import message_filters
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32, Float64MultiArray 


x, y, psi = 0,0,0
lidardata = [99] * 360
Doking_state=50




ax = plt.subplot(221, polar = True)
ax1=plt.subplot(223,xlim=(-60,60),ylim=(0,1000))
ax2=plt.subplot(222,xlim=(-60,60),ylim=(0,1000))


data_range=np.linspace(0,2*np.pi, 360)
def update(data1, data2, data3):
    global x, y, psi, lidardata,Pos
    x, y = data1.data[0], data1.data[1]
    psi = data2.data
    lidardata = data3.data
    Pos=[x,y]

def animate(i):
    global psi
    Angle_Line=50
    Xlist=[]
    Ylist=[]
    Doking_Candi=[]
    Point_Plot=[0]*360
    if psi>180:
        psi-=360
    ld=list(lidardata)
    Atan_List=[]
    for i in range(61, 300, 1):
        ld[i] = 0
    for i in range(-60,60):
        Xlist.append(int(ld[i]*math.sin(i*math.pi/180)))
        Ylist.append(int(ld[i]*math.cos(i*math.pi/180)))
            
    






    Pre_Atan=0
    List1=[]
    for i in range(len(Xlist)-1):
        if Xlist[i]-Xlist[i+1]!=0:
            if math.atan2((Ylist[i+1]-Ylist[i]),(Xlist[i+1]-Xlist[i]))>0:
                Atan_List.append((math.atan2((Ylist[i+1]-Ylist[i]),(Xlist[i+1]-Xlist[i])))*180/math.pi)
            else:
                Atan_List.append((math.atan2((Ylist[i+1]-Ylist[i]),(Xlist[i+1]-Xlist[i])))*180/math.pi+360)
        else:
            Atan_List.append(psi)





    Middle_Satand=0
    for i in range(len(Atan_List)):
        if psi<0:
            if Atan_List[i]<360+psi-Angle_Line:
                ld[i-60]=0
                List1.append(i)
                Middle_Satand=360+psi-Angle_Line
        else:
            if Atan_List[i]>abs(psi)+Angle_Line-10:
                ld[i-60]=0
                List1.append(i)
                Middle_Satand=abs(psi)




    
    for i in range(-60,60):
        if ld[i]-ld[i+1]>Doking_state:
            Doking_Candi.append([i,ld[i]])
            Doking_Candi.append([i+1,ld[i+1]])
        elif ld[i+1]-ld[i]>Doking_state:
            Doking_Candi.append([i,ld[i]])
            Doking_Candi.append([i+1,ld[i+1]])





    Doking_Candi_Remove=[]
    for i in range(len(Doking_Candi)-1):
        if abs(Doking_Candi[i][0]-Doking_Candi[i+1][0])<2 and abs(Doking_Candi[i][1]-Doking_Candi[i+1][1])<20:
            Doking_Candi_Remove.append(i+1)
    for i in range(len(Doking_Candi_Remove)-1):
        Doking_Candi.pop(Doking_Candi_Remove[i]-i)
    for i in range(len(Doking_Candi)):
        Point_Plot[Doking_Candi[i][0]]=Doking_Candi[i][1]
    

    Point_x=[]
    Point_y=[]
    Middle_X=[]
    Middle_Y=[]
    Range_X=[]
    Middle_Plot=[0]*360
    for i in range(-30,30):
        Xlist.append(int(ld[i]*math.sin(i*math.pi/180)))
        Ylist.append(int(ld[i]*math.cos(i*math.pi/180)))
    for i in range(len(Doking_Candi)):
        Point_x.append(Doking_Candi[i][1]*math.sin(Doking_Candi[i][0]*math.pi/180))
        Point_y.append(Doking_Candi[i][1]*math.cos(Doking_Candi[i][0]*math.pi/180))
    # for i in range(len(Point_Y)-1):
    #     if math.sqrt(Point_X[i]*Point_X[i]+Point_Y[i]*Point_Y[i])-5<ld[math.atan2(Point_X[i], Point_Y[i])]<math.sqrt(Point_X[i]*Point_X[i]+Point_Y[i]*Point_Y[i])+5:
    #         Middle_X.append((Point_X[i]+Point_X[i+1])/2)
    #         Middle_Y.append((Point_Y[i]+Point_Y[i+1])/2)
    for i in range(len(Point_y)-1):
        if math.sqrt(((Point_x[i]+Point_x[i+1])/2)*((Point_x[i]+Point_x[i+1])/2)+((Point_y[i]+Point_y[i+1])/2)*((Point_y[i]+Point_y[i+1])/2))-18<ld[int((math.atan2((Point_x[i]+Point_x[i+1])/2, (Point_y[i]+Point_y[i+1])/2))*180/math.pi)]<math.sqrt(((Point_x[i]+Point_x[i+1])/2)*((Point_x[i]+Point_x[i+1])/2)+((Point_y[i]+Point_y[i+1])/2)*((Point_y[i]+Point_y[i+1])/2))+18:
            Middle_X.append((Point_x[i]+Point_x[i+1])/2)
            Middle_Y.append((Point_y[i]+Point_y[i+1])/2)
            Middle_Plot[int((math.atan2((Point_x[i]+Point_x[i+1])/2, (Point_y[i]+Point_y[i+1])/2))*180/math.pi)]=ld[int((math.atan2((Point_x[i]+Point_x[i+1])/2, (Point_y[i]+Point_y[i+1])/2))*180/math.pi)]
        print(math.atan2((Point_x[i]+Point_x[i+1])/2, (Point_y[i]+Point_y[i+1])/2))
        print(int((math.atan2((Point_x[i]+Point_x[i+1])/2, (Point_y[i]+Point_y[i+1])/2))*180/math.pi))









    # msg = Float32()
    # msg.data = 30   
    # pub.publish(msg)
        









    ax.clear()
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.plot(data_range,ld,'o',color='r',markersize=1)
    ax.fill(data_range,ld,"0.5")
    ax.plot(0,0,'s',color='blue',markersize=15)
    ax.plot(data_range,Point_Plot,'s',color='purple')
    ax.plot(data_range,Middle_Plot,'o',color='green',markersize=3)
    ax.set_thetamin(-60)
    ax.set_thetamax(60)
    ax.set_rmax(500)
    ax.set_rmin(0)
    ax1.clear()
    ax2.clear()


    ax2.axis([-65,65,0,600])
    ax2.plot(range(-60,59),Atan_List,'bo', markersize = 2)
    # ax2.plot(Doking_Candi,Doking_Candi_2,'ro', markersize = 2)
    ax1.axis([-500,500,0,1000])
    ax1.plot(Xlist,Ylist,'ro',markersize=2)
    ax1.plot(Point_x,Point_y,'bo',markersize=3)
    ax1.plot(Middle_X,Middle_Y,'o',color='purple',markersize=3)
    

if __name__=="__main__":
    rospy.init_node("YPathPlan")
    #pub=rospy.Publisher("/test", Float32)
    sub1=message_filters.Subscriber("GPSData" , Float64MultiArray)
    sub2=message_filters.Subscriber("IMUData" , Float32)
    sub3=message_filters.Subscriber("LidarData" , Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3], 10, 0.1, allow_headerless=True)
    mf.registerCallback(update)
    ani=FuncAnimation(plt.gcf(),animate, interval=1000)
    plt.show()

