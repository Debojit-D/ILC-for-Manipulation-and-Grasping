import rospy
import rostopic
from iiwa_msgs.msg import JointPosition, JointVelocity, CartesianPose , CartesianQuantity, JointTorque
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from iiwa_msgs.srv import TimeToDestination, TimeToDestinationRequest, TimeToDestinationResponse
import time
import numpy as np
import pandas as pd
from numpy.linalg import norm
import matplotlib.pyplot as plt
from kuka_kinematics import forward_map, Jacobian_matrix, Jacobian_dot
########################################################################################################
# Time Trials
dt = 1/100
T=2
freq=1/dt
n=int((2/dt))
t= np.arange(0, T , dt)
BL=100
PT=100
WS=50
RP=PT
trials=BL+PT+WS+RP
########################################################################################################
#Controller Properties
yd=np.zeros(len(t))
yi=0
yf=0.2
for k in range(0, len(t) - 1):
    yd[k]= yi + (yf-yi)*(10*(t[k]/T)**3 - 15*(t[k]/T)**4 + 6*(t[k]/T)**5)
yd[k+1]=yd[k]
zd=np.ones(len(t))*0
XD=np.vstack((yd, zd))

#
def sat_u(v):
    saturation_min = -10
    saturation_max = 10
    v_saturated = np.clip(v, saturation_min, saturation_max)
    return v_saturated

#System Parametrs
X=np.zeros((2,len(t)))
U=np.zeros((2,len(t)))
b=1
s=10
Ac=np.array([[(-s/b), 0],[0 ,(-s/b)]])
A=np.eye(2) + Ac*dt
Bc=np.array([[1/b,0], [0,1/b]])
B=Bc*dt
C=np.eye(2)

CAB=np.matmul(C,B)
inv_CAB=np.linalg.pinv(CAB)
# Twin properties
u_ilc=np.zeros((2,len(t)))
u_ff=np.zeros((2,len(t)))
X_pred=np.vstack((X[0,:]*0,X[1,:]*0))
err=np.zeros(trials)
unrm=np.zeros(trials)

Fz=np.ones((1,len(t)))*0
Fext=np.vstack((np.zeros((1,len(t))),Fz))
PG=-3

def controller(XD,X,X_pred,CAB,inv_CAB,u_ilc,u_ff,U,t,n,Fext,PG):
    # Initial Conditions
    X[0,0]=0
    X[1,0]=0

    if n>BL and n <=PT+BL:
        Fz=np.ones((1,len(t)))*PG
        Fext=np.vstack((np.zeros((1,len(t))),Fz))

    if n>PT+BL and n <= PT+BL+WS :
        Fz=np.zeros((1,len(t)))*0
        Fext=np.vstack((np.zeros((1,len(t))),Fz))

    if n> PT+BL+WS:
        Fz=np.ones((1,len(t)))*PG
        Fext=np.vstack((np.zeros((1,len(t))),Fz))

    #Twin
    for k in range(0,len(t)-1):
        X[:,k+1]= np.matmul(A,X[:,k])  + np.matmul(B,U[:,k] + Fext[:,k] )
  
    # Errors
    XM=np.vstack((X[0,:],X[1,:]))
    TE=(XD-XM)
    SPE=(XM-X_pred)
    CBU=np.matmul(CAB,U)
    MOE=np.vstack((CBU[0,:],CBU[1,:]))-XM
    MOEC=np.matmul(inv_CAB,MOE)
    #Controller

    u_ilc= 0.9999*u_ilc + 0.9*np.tanh(TE)
    X_pred=0.5*X_pred + 0.9*np.tanh(SPE)
    u_ff=0.75*u_ff + 0.05*MOEC
    u_ilc=sat_u(u_ilc)
    u_ff=sat_u(u_ff)
    U=(1*u_ilc+ 1*(u_ff))

    #Slepp for changing weight
    if n==BL+1:
        rospy.sleep(120)

    if n==BL+PT+1:
        rospy.sleep(120)

    if n==BL+PT+WS+1:
        rospy.sleep(120)

    
    return X,X_pred,u_ilc,u_ff,U



###########################################################################################################
# Initial Position
rospy.init_node('set_CartesianPose',anonymous=False)
msg = rospy.wait_for_message("/iiwa/state/CartesianPose", CartesianPose)
init_position = [msg.poseStamped.pose.position.x, msg.poseStamped.pose.position.y, msg.poseStamped.pose.position.z]
init_orn = [msg.poseStamped.pose.orientation.x, msg.poseStamped.pose.orientation.y, msg.poseStamped.pose.orientation.z, msg.poseStamped.pose.orientation.w]
print(init_position)

#Execution
#define a publisher
pub = rospy.Publisher("/iiwa/command/CartesianPose",PoseStamped, queue_size=1)
time.sleep(1)
pose0 = PoseStamped()
pose0.header.stamp = rospy.Time.now()
pose0.header.frame_id = "iiwa_link_0" 
time_now = []
time_diff = []
time_elapsed = []
time_start = rospy.get_time()
msd_position=np.zeros((len(t),3))
rate  = rospy.Rate(freq)
###########################################################################################################
#Info Logger
th_current = np.zeros((1,7))
x_current = np.zeros((1,3))
f_current = np.zeros((1,3))
def callback_p(data):
    global  th_current,x_current,J
    th_m=[np.array([data.position.a1,data.position.a2,data.position.a3,data.position.a4,data.position.a5,data.position.a6,data.position.a7])]
    th_current = np.append(th_current ,th_m,axis=0)
    x_m = forward_map(th_m[0])
    J=Jacobian_matrix(th_m[0])
    xm_a=[np.array([x_m[0],x_m[1],x_m[2]])]
    x_current = np.append(x_current, xm_a,axis=0)
 
def callback_t(data):
    global  f_current
    tau=[np.array([data.torque.a1,data.torque.a2,data.torque.a3,data.torque.a4,data.torque.a5,data.torque.a6,data.torque.a7])]
    JT=J.transpose()
    JTI=np.linalg.pinv(JT)
    tau_m=tau[0]
    F=np.matmul(JTI,tau_m)
    fm_a=[np.array([F[0],F[1],F[2]])]
    f_current = np.append(f_current, fm_a,axis=0)

FP='/home/rbccps-kuka/iiwa_stack_ws/src/scripts/Adaptive_ctrl/EXP_DATA_7/'
#########################################################################################################

for n in range(0,trials):
    X,X_pred,u_ilc,u_ff,U=controller(XD,X,X_pred,CAB,inv_CAB,u_ilc,u_ff,U,t,n,Fext,PG)
    ye=X[0,:]+init_position[1]
    ze=X[1,:]+init_position[2]
    rospy.sleep(1)
    pose0.pose.position = Point(x=init_position[0], y=init_position[1], z=init_position[2])
    pose0.pose.orientation = Quaternion(*init_orn)
    pub.publish(pose0)
    rospy.sleep(2)
    jpos=rospy.Subscriber("/iiwa/state/JointPosition", JointPosition, callback_p)
    jtorque=rospy.Subscriber("/iiwa/state/ExternalJointTorque", JointTorque, callback_t)
    rospy.sleep(0.5)
    for k in range(int(freq*T)):
        time_now.append(rospy.get_time())
        time_elapsed.append(time_now[k]-time_start)
    
        if k == 0:
            time_diff.append(time_elapsed[k])
        else:
            time_diff.append(time_now[k]-time_now[k-1])
        try:
            if (np.min(ye) > init_position[1]-0.35) and  (np.max(ye) < init_position[1]+0.35) and  (np.min(ze) > init_position[2]-0.35) and  (np.max(ze) < init_position[2]+0.35):
                pose0.pose.position = Point(x=init_position[0], y=ye[k], z=ze[k])
                pose0.pose.orientation = Quaternion(*init_orn)
                pub.publish(pose0)
                # print(ye[k])
                # print(',')
                # print(ze[k])
                print(np.array([k,n]))
            else:
                print('BAD OUTPUT')

        except rospy.ROSInterruptException:
            pass
            break
   
        rate.sleep()

    rospy.sleep(1)
    jpos.unregister()
    jtorque.unregister()
    time = pd.DataFrame.from_dict({'time_diff':time_diff, 'time':time_elapsed})
    pos_th = pd.DataFrame.from_dict({'Pos1':th_current[:,0],'Pos2':th_current[:,1],'Pos3':th_current[:,2],'Pos4':th_current[:,3],'Pos5':th_current[:,4],'Pos6':th_current[:,5],'Pos7':th_current[:,6]})
    pos_cs=pd.DataFrame.from_dict({'X':x_current[:,0],'Y':x_current[:,1],'Z':x_current[:,2]})
    F_cs=pd.DataFrame.from_dict({'X':f_current[:,0],'Y':f_current[:,1],'Z':f_current[:,2]})
    time.to_csv(FP+'time_s'+str(n+1)+'.csv')
    pos_th.to_csv(FP+'posth_s'+str(n+1)+'.csv')
    pos_cs.to_csv(FP+'pos_cs'+str(n+1)+'.csv')
    F_cs.to_csv(FP+'force_cs'+str(n+1)+'.csv')
    th_current = np.zeros((1,7))
    x_current = np.zeros((1,3))
    f_current = np.zeros((1,3))
    
rospy.sleep(2)
pose0.pose.position = Point(x=init_position[0], y=init_position[1], z=init_position[2])
pose0.pose.orientation = Quaternion(*init_orn)
pub.publish(pose0)
rospy.sleep(2)




# tor1.to_csv('tor1_12.csv')