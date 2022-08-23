import rospy
import math
import message_filters
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float32, Float64MultiArray 
import random





x, y, psi = 0,0,0
lidardata = [0] * 360
Xlist=[]
Ylist=[]
GoalList=[]
fig, ax = plt.subplots(1,1, figsize=(10,5))
minimum=100
rangeY=1001
rangeX=500
Lidardata=[]
x=random.randrange(500)
y=random.randrange(500)
k=0
for i in range(360):
    Lidardata.append(object)






def update(data1, data2, data3):
    global x, y, psi, lidardata
    x, y = data1.data[0], data1.data[1]
    psi = data2.data
    lidardata = data3.data



class JPS():
    def __init__(self,Goal):
        #print("Start")
        self.Xnum=100
        self.Ynum=100
        self.obstacle=np.zeros((self.Xnum,self.Ynum))
        

    def setting(self):
        self.plot1=np.zeros((self.Ynum,self.Xnum))
        self.visited=np.copy(self.plot1)
        self.preNode=[[(11,11) for i in range(self.plot1.shape[1])] for j in range(self.plot1.shape[0])]
        #print(preNode)
        self.Hcost=np.copy(self.plot1)
        self.Gcost=np.copy(self.plot1)
        self.Mcost=np.copy(self.plot1)
        self.cost=[]
        self.List1=[]
        self.List2=[]
        self.List3=[]
        self.List4=[]
        self.ImportL=[]
        self.ImportLnotSame=[]
        self.ListCost=[]
        self.PickList=[]
        self.inputY=0
        self.inputX=0
        self.preX=0
        self.PreY=0
        self.dy=[-1,-1,0,1,1,1,0,-1]
        self.dx=[0,1,1,1,0,-1,-1,-1]
        self.endY=20
        self.endX=20
        self.Distance=math.sqrt(self.endY*self.endY+self.endX*self.endX)
        self.StartY=1  
        self.StartX=1
        self.pathB=[]
        self.nowX=0
        self.nowY=0
        


    def go1(self,y,x,i,PY,PX): ########################SearchN탐색
        searchN=0
        PreX=PX
        PreY=PY
        ny=y
        nx=x

        while (ny >= 0 and ny < self.plot1.shape[0] and nx >= 0 and nx < self.plot1.shape[1]):
                ny=ny+self.dy[i]
                nx=nx+self.dx[i]
                if ( ny >= 0 and ny < self.plot1.shape[0] and nx >= 0 and nx < self.plot1.shape[1] and self.obstacle[ny, nx]== 0):
                    searchN+=1
                    if self.preNode[ny][nx]==(11,11):
                        self.preNode[ny][nx]=(y,x)
                        #visited[ny][nx]=1

                else:
                    ny=ny-self.dy[i]
                    nx=nx-self.dx[i]
                    break
                if(ny==self.endY and nx==self.endX):
                    self.ImportLnotSame.insert(0,(ny,nx))
                    break
                #visited[ny][nx]=1
        #visited[ny][nx]=0
        if (ny >= 0 and ny < self.plot1.shape[0] and nx >= 0 and nx < self.plot1.shape[1]):
            ny=y
            nx=x
            if i==0:
                self.List1.append((searchN,(ny,nx),(PreY,PreX)))
            if i==2:
                self.List2.append((searchN,(ny,nx),(PreY,PreX)))
            if i==4:
                self.List3.append((searchN,(ny,nx),(PreY,PreX)))
            if i==6:
                self.List4.append((searchN,(ny,nx),(PreY,PreX)))
                self.List4.append((searchN,(ny,nx),(PreY,PreX)))
            if(ny==self.endY and nx==self.endX):
                ##self.ImportLnotSame.insert(0,(ny,nx))
                self.ImportLnotSame.insert(0,(ny,nx))


    def go3(self,y,x,i,PY,PX): #####################################go1함수에서 List들에 append가 아닌 insert를 넣어야하는 경우를 고려한 함수, 그 이외의 차이X
        searchN=0
        PreX=PX
        PreY=PY
        ny=y
        nx=x  
        while ny >= 0 and ny < self.plot1.shape[0] and nx >= 0 and nx < self.plot1.shape[1]:
                ny=ny+self.dy[i]
                nx=nx+self.dx[i]
                if ( ny >= 0 and ny < self.plot1.shape[0] and nx >= 0 and nx < self.plot1.shape[1] and self.obstacle[ny, nx]== 0):              
                        searchN+=1
                        if self.preNode[ny][nx]==(11,11):
                            self.preNode[ny][nx]=(y,x)
                        #visited[ny][nx]=1

                else:
                    ny=ny-self.dy[i]
                    nx=nx-self.dx[i]
                    break
        #visited[ny][nx]=0
        if (ny >= 0 and ny < self.plot1.shape[0] and nx >= 0 and nx < self.plot1.shape[1]):
            ny=y
            nx=x  
            if i==0:
                self.List1.insert(0,(searchN,(ny,nx),(PreY,PreX)))
            if i==2:
                self.List2.insert(0,(searchN,(ny,nx),(PreY,PreX)))
            if i==4:
                self.List3.insert(0,(searchN,(ny,nx),(PreY,PreX)))
            if i==6:
                self.List4.insert(0,(searchN,(ny,nx),(PreY,PreX)))
            if(ny==self.endY and nx==self.endX):
                ##self.ImportLnotSame.insert(0,(ny,nx))
                self.ImportLnotSame.insert(0,(ny,nx))

    def go2(self,y,x,i): #################################################################대각선 방향으로 탐색
        PreX=x
        PreY=y
        ny=y
        nx=x
        while (ny >= 0 and ny < self.plot1.shape[0] and nx >= 0 and nx < self.plot1.shape[1]):
                ny+=self.dy[i]
                nx+=self.dx[i]
                if (ny>=0 and ny < self.plot1.shape[0] and nx >= 0 and nx < self.plot1.shape[1] and self.obstacle[ny, nx]== 0):
                    # if (visited[ny, nx] == 1):
                    #     break        
                    if self.preNode[ny][nx]==(11,11):
                        self.preNode[ny][nx]=(PreY,PreX)
                    if i==1:
                        self.go1(ny,nx,0,PreY,PreX)
                        self.go1(ny,nx,2,PreY,PreX)
                    if i==3:
                        #print('a')
                        self.go3(ny,nx,2,PreY,PreX)
                        self.go1(ny,nx,4,PreY,PreX)
                    if i==5:
                        self.go3(ny,nx,4,PreY,PreX)
                        self.go1(ny,nx,6,PreY,PreX)
                    if i==7:
                        self.go3(ny,nx,6,PreY,PreX)
                        self.go3(ny,nx,0,PreY,PreX)
                    if(ny==self.endY and nx==self.endX):
                        ##self.ImportLnotSame.insert(0,(ny,nx))
                        self.ImportLnotSame.insert(0,(ny,nx))
                    #visited[ny][nx]=1
                elif (ny>=0 and ny < self.plot1.shape[0] and nx >= 0 and nx < self.plot1.shape[1] and self.obstacle[ny, nx]== 1):
                    if i==1:
                        self.List1.append((-1,(ny,nx),(PreY,PreX)))
                        self.List2.append((-1,(ny,nx),(PreY,PreX)))
                    elif i==3:
                        self.List2.insert(0,(-1,(ny,nx),(PreY,PreX)))
                        self.List3.append((-1,(ny,nx),(PreY,PreX)))
                    elif i==5:
                        self.List3.insert(0,(-1,(ny,nx),(PreY,PreX)))
                        self.List4.append((-1,(ny,nx),(PreY,PreX)))
                    else:
                        self.List4.insert(0,(-1,(ny,nx),(PreY,PreX)))
                        self.List1.insert(0,(-1,(ny,nx),(PreY,PreX)))
                    break
                else:
                    ny=ny-self.dy[i]
                    nx=nx-self.dx[i]
                    if self.preNode[ny][nx]==(11,11):
                        self.preNode[ny][nx]=(PreY,PreX)
                        #self.visited[ny][nx]=1
                    # ImportLX.append(nx)
                    # ImportLY.append(ny)
                    break

    def jps(self,y,x):   #######################jps실행
        self.visited[y][x]=1
        if(y >= 0 and y < self.plot1.shape[0] and x >= 0 and x < self.plot1.shape[1]):
            self.go1(y,x,0,y,x)
            self.go1(y,x,2,y,x)
            self.go1(y,x,4,y,x)
            self.go1(y,x,6,y,x)
            self.go2(y,x,1)
            self.go2(y,x,3)
            self.go2(y,x,5)
            self.go2(y,x,7)

    def ImportantNode(self): #####################################################주변 노드 간의 SearchN의 길이 값들의 차를 구하는 함수
        for i in range(len(self.List1)-1):
            if self.List1[i][0]-self.List1[i+1][0]>1 and self.visited[self.List1[i+1][1][0]-self.List1[i+1][0]-1][self.List1[i][1][1]]==0 and self.obstacle[self.List1[i+1][1][0]-self.List1[i+1][0]-1][self.List1[i][1][1]]==0:
                self.ImportL.append((self.List1[i+1][1][0]-self.List1[i+1][0]-1,self.List1[i][1][1]))
            elif self.List1[i][0]-self.List1[i+1][0]<-1 and self.visited[self.List1[i][1][0]-self.List1[i][0]-1][self.List1[i+1][1][1]]==0 and self.obstacle[self.List1[i][1][0]-self.List1[i][0]-1][self.List1[i+1][1][1]]==0:
                self.ImportL.append((self.List1[i][1][0]-self.List1[i][0]-1,self.List1[i+1][1][1]))
                #print(i)
        for i in range(0,len(self.List1)):
            self.List1.pop(0)
           
        for i in range(len(self.List2)-1):
            #print(i)
            if self.List2[i][0]-self.List2[i+1][0]>1 and self.visited[self.List2[i][1][0]][self.List2[i+1][1][1]+self.List2[i+1][0]+1]==0 and self.obstacle[self.List2[i][1][0]][self.List2[i+1][1][1]+self.List2[i+1][0]+1]==0:
                self.ImportL.append((self.List2[i][1][0],self.List2[i+1][1][1]+self.List2[i+1][0]+1))
            elif self.List2[i][0]-self.List2[i+1][0]<-1 and self.visited[self.List2[i+1][1][0]][self.List2[i][1][1]+self.List2[i][0]+1]==0 and self.obstacle[self.List2[i+1][1][0]][self.List2[i][1][1]+self.List2[i][0]+1]==0:
                self.ImportL.append((self.List2[i+1][1][0],self.List2[i][1][1]+self.List2[i][0]+1))
        for i in range(0,len(self.List2)):
            self.List2.pop(0)
       
        for i in range(len(self.List3)-1):
            if self.List3[i][0]-self.List3[i+1][0]>1 and self.visited[self.List3[i+1][1][0]+self.List3[i+1][0]+1][self.List3[i][1][1]]==0 and self.obstacle[self.List3[i+1][1][0]+self.List3[i+1][0]+1][self.List3[i][1][1]]==0:
                self.ImportL.append((self.List3[i+1][1][0]+self.List3[i+1][0]+1,self.List3[i][1][1]))
            elif self.List3[i][0]-self.List3[i+1][0]<-1 and self.visited[self.List3[i][1][0]+self.List3[i][0]+1][self.List3[i+1][1][1]]==0 and self.obstacle[self.List3[i][1][0]+self.List3[i][0]+1][self.List3[i+1][1][1]]==0:
                self.ImportL.append((self.List3[i][1][0]+self.List3[i][0]+1,self.List3[i+1][1][1]))
        for i in range(0,len(self.List3)):
            self.List3.pop(0)

        for i in range(len(self.List4)-1):
            if self.List4[i-1][0]-self.List4[i][0]>1 and self.visited[self.List4[i-1][1][0]][self.List4[i][1][1]-self.List4[i][0]-1]==0 and self.obstacle[self.List4[i-1][1][0]][self.List4[i][1][1]-self.List4[i][0]-1]==0:
                self.ImportL.append((self.List4[i-1][1][0],self.List4[i][1][1]-self.List4[i][0]-1))
            elif self.List4[i-1][0]-self.List4[i][0]<-1 and self.visited[self.List4[i][1][0]][self.List4[i-1][1][1]-self.List4[i-1][0]-1]==0 and self.obstacle[self.List4[i][1][0]][self.List4[i-1][1][1]-self.List4[i-1][0]-1]==0:
                self.ImportL.append((self.List4[i][1][0],self.List4[i-1][1][1]-self.List4[i-1][0]-1))
        for i in range(0,len(self.List4)):
            self.List4.pop(0)
        for value in self.ImportL:
            if value not in self.ImportLnotSame:
                if self.visited[value[0]][value[1]]==0:
                    self.ImportLnotSame.append(value)
        for i in range(0,len(self.ImportLnotSame)-1):
            if self.ImportLnotSame[i]==(self.endY,self.endX):
                self.ImportLnotSame[0],self.ImportLnotSame[i]=self.ImportLnotSame[i],self.ImportLnotSame[0]

    def search(self,y,x):
        if(y >= 0 and y < self.plot1.shape[0] and x >= 0 and x < self.plot1.shape[1]):  
            a = abs(self.endY - (y))
            b = abs(self.endX - (x))
            if(a >= b):
                self.Hcost[y][x]= (b*14 + (a-b)*10)*15
            else:
                self.Hcost[y][x]= a*14 + (b-a)*15
            c=abs(self.preNode[y][x][0]-y)
            d=abs(self.preNode[y][x][1]-x)
            if(c >= d):
                self.Gcost[y][x]= (d*14 + (c-d)*10)
            else:
                self.Gcost[y][x]= (c*14 + (d-c)*10)
            e=abs(self.StartY-y)
            f=abs(self.StartX-x)
            if(e >= f):
                self.Mcost[y][x]= (f*14 + (e-f)*10)
            else:
                self.Mcost[y][x]= (e*14 + (f-e)*10)
            # e=abs(self.StartY-y)
            # f=abs(self.StartX-x)
            # if(e >= f):
            #     self.Gcost[y][x]= f*14 + (e-f)*10
            # else:
            #     self.Gcost[y][x]= f*14 + (e-f)*10
        self.cost.append(self.Hcost[y][x]+ self.Gcost[y][x]+self.Mcost[y][x])
        ###############################################################################################cost값계산

    def RealMain(self,y,x):
        self.jps(y,x)
        self.ImportantNode()
        inputY=self.ImportLnotSame[0][0]
        inputX=self.ImportLnotSame[0][1]
        self.ImportLnotSame
        self.PickList.append((inputY,inputX))
        PreX=x
        PreY=y
        while inputY!=self.endY or inputX!=self.endX:
            self.jps(inputY,inputX)
            self.ImportantNode()
            for i in range(len(self.ImportLnotSame)):
                self.search(self.ImportLnotSame[i][0],self.ImportLnotSame[i][1])
                self.ListCost.append(self.cost[i])
            for i in range(len(self.ImportLnotSame)-1):
                if self.ListCost[i]>=self.ListCost[i+1]:
                    self.ListCost[i+1],self.ListCost[i]=self.ListCost[i],self.ListCost[i+1]
                    self.ImportLnotSame[i+1],self.ImportLnotSame[i]=self.ImportLnotSame[i],self.ImportLnotSame[i+1]
                    for j in range(i,0,-1):#
                        if self.ListCost[j]<=self.ListCost[j-1]:
                            self.ListCost[j],self.ListCost[j-1]=self.ListCost[j-1],self.ListCost[j]
                            self.ImportLnotSame[j],self.ImportLnotSame[j-1]=self.ImportLnotSame[j-1],self.ImportLnotSame[j]###############중복되는 점 고려를 위한 부분  
            PreY=inputY
            PreX=inputX      
            #print(PreX)          
            inputY=self.ImportLnotSame[0][0]
            inputX=self.ImportLnotSame[0][1]
            self.nowX=self.ImportLnotSame[0][1]
            self.nowY=self.ImportLnotSame[0][0]
            self.PickList.append((inputY,inputX))
            self.ImportLnotSame.pop(0)
            if inputY==self.endY and inputX==self.endX:
                self.visited[inputY][inputX]=1  
            # print(self.ImportLnotSame)  
    # RealMain(0,0)
    def backtracking(self,y,x):
        #print(self.preNode)
        ny=self.preNode[y][x][0]
        #print(ny)
        nx=self.preNode[y][x][1]
        #print("ny:%d  /\   nx:%d",(ny,nx))
        self.plot1[ny][nx]=88
        self.plot1[self.endY][self.endY]=88
        self.pathB.insert(0,(ny,nx))
        if ny==self.StartY and nx==self.StartX:
            return 100
        #print(self.pathB)

        self.backtracking(ny,nx)
   
    def Run(self):
        self.RealMain(self.StartY,self.StartX)
        self.pathB.append((self.endY,self.endX))
        #print(self.pathB)
        #print(self.obstacle)
        self.backtracking(self.endY,self.endX)

class WayPoint():
    def __init__(self,ratio,shipPoint):
        self.ratio=ratio
        self.shipPointX=shipPoint[1]
        self.shipPointY=shipPoint[0]
    def waypoint(self,wayp):
        self.realPX=(wayp[1]-50)/self.ratio+self.shipPointX
        self.realPY=wayp[0]/self.ratio+self.shipPointY



def animate(i):

    Xlist = []
    Ylist = []
    GoalList.append((20,20))
    ld=np.array(lidardata)/2
    rangeX2=int((rangeX/minimum)*10)
    #print(rangeX2)
    rangeY2=int((rangeY/minimum)*10)
    map2=np.zeros([rangeY2,rangeX2])
    #print(map2.shape)
    for i in range(61, 300, 1):
        ld[i] = 0
    for i in range(0,360):
        Xlist.append(int(ld[i]*math.sin(i*math.pi/180)/minimum*10))
        Ylist.append(int(ld[i]*math.cos(i*math.pi/180)/minimum*10))
        map2[int(ld[i]*math.cos(i*math.pi/180)/minimum*10)][int(ld[i]*math.sin(i*math.pi/180)/minimum*10)]=1
    #print(Xlist)
    ax.clear()
    ax.grid()
    ax.axis([int(-500/minimum*10),int(500/minimum*10), 0, int(500/minimum*10)])
    ax.plot(Xlist,Ylist, "o", markersize = 2)
    jps=JPS(10)
    jps.Xnum=rangeX2
    jps.Ynum=rangeY2
    jps.obstacle=map2
    jps.setting()
    global k
    print(k)
    Xdiffer=GoalList[k][0]
    Ydiffer=GoalList[k][1]
    k+=1
    GoalList.pop(-1)
    GoalAngle=math.atan2(Xdiffer, Ydiffer)###############x,y반대로 넣어야함
    NowAngle=psi
    # print(psi)
    # print(GoalAngle)
    AngleDiffer=GoalAngle-NowAngle
    jps.endY=int(jps.Distance*math.sin(AngleDiffer))
    jps.endX=int(jps.Distance*math.cos(AngleDiffer))
    print((jps.endX,jps.endY))
    jps.Run()
    finalPath = np.array(jps.pathB).transpose()
    print(finalPath)
    ax.plot(finalPath[1], finalPath[0])
    ratio=rangeX2/rangeX
    way=WayPoint(ratio, (y,x))
    way.waypoint(finalPath)

    # Xdiffer=way.realPX[-1]-x
    # Ydiffer=way.realPY[-1]-y
    # GoalAngle=math.atan2(Xdiffer, Ydiffer)###############x,y반대로 넣어야함
    # NowAngle=psi
    # AngleDiffer=GoalAngle-NowAngle
    # jps.endY=jps.Distance*math.sin(AngleDiffer)
    # jps.endX=jps.Distance*math.cos(AngleDiffer)
    print(((x-500,y),(way.realPX[-1],way.realPY[-1])))
    GoalList.append((way.realPX[-1],way.realPY[-1]))




    #jps=JPS(10)

    #for i in range(360):
    #    jps.obstacle[int((Ylist[i]+500)/2)][int((Xlist[i]+500)/2)]=1
    #jps.Run()
    #print(jps.obstacle)
    #ax.plot(jps.pathB[1],jps.pathB[0],"ro",markersize=10)
    #print(max(ld))

        









if __name__=="__main__":
    rospy.init_node("YPathPlan2")

    sub1=message_filters.Subscriber("GPSData",Float64MultiArray)
    sub2=message_filters.Subscriber("IMUdata",Float32)
    sub3=message_filters.Subscriber("LidarData",Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3], 10, 0.1, allow_headerless=True)
    mf.registerCallback(update)
    ani=FuncAnimation(plt.gcf(),animate, interval=200)
    plt.show()
