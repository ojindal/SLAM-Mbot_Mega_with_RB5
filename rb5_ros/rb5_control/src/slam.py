#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from rospy.numpy_msg import numpy_msg
from april_detection.msg import AprilTagDetectionArray
from tf.transformations import quaternion_multiply
import numpy as np
import time
import math
from numpy import asarray
from numpy import savetxt

scale = 1
flag = True

# Maps tag id to the index in sigma matrix
id2ind_sigma = {}

# Maps tag id to the index in mu matrix
id2ind_mu = {}

global mu
global sigma
global traj
traj = []
# Sigma initiation
sigma = np.random.uniform(low=0.001, high=0.02, size=(27,27))
sig_r = 0.01*np.identity(3)
sig_t = 20*np.identity(3)
sigma[:3,:3] = sig_r
for i in range(8):
    sigma[3*i:3*i+3,3*i:3*i+3] = sig_t

# Mu initiation
mu = np.zeros((27,))
print('mu.shape', mu.shape)

mu_ins = np.zeros((27,))

z = np.zeros((27,))
print('mu.shape', mu.shape)

def r2w_w2r(th_prior):
    real = np.array([[np.sin(th_prior), np.cos(th_prior), 0],
       [-np.cos(th_prior), np.sin(th_prior), 0],
       [0, 0, 1]])
    
    # inv = np.linalg.inv(real)

    return real

# H initiation
def h_mat(n_tags):
    ans = np.zeros((3*n_tags, 3*(n_tags+1)))
    ans[:,:3] = np.eye(ans[:,:3].shape[0])
    for i in range(n_tags):
        ans[3*i:3*i+3,:3] = -1*np.eye(3)
    return ans

def temp_sig(sigma, ids):
    l = len(ids)

    # Dictionary that maps the index of temp matrix to index in permanent matrix
    indexes = {}

    ans = np.zeros((3*(l+1),3*(l+1)))>3
    ind1, ind2 = 3, 3
    for id1 in ids:
        for id2 in ids:
            i1, i2 = id2ind_sigma[id1], id2ind_sigma[id2]
            ans[ind1:ind1+3, ind2:ind2+3] = sigma[i1:i1+3, i2:i2+3]
            ans[ind2:ind2+3, ind1:ind1+3] = sigma[i2:i2+3, i1:i1+3]
            indexes[ind1] = i1
            indexes[ind2] = i2
            ind1+=3
            ind2+=3
    assert (np.count_nonzero(ans == False) == 0)

    # array and map
    return ans, indexes

def sig_temp2perm(sigma, sigma_temp, indexes):
    for ind1 in indexes.keys():
        for ind2 in indexes.keys():
            i1, i2 = indexes[ind1], indexes[ind2]
            sigma[i1:i1+3, i2:i2+3] = sigma_temp[ind1:ind1+3, ind2:ind2+3]
            sigma[i2:i2+3, i1:i1+3] = sigma_temp[ind2:ind2+3, ind1:ind1+3]
    return sigma

def temp_mu(mu, ids):
    l = len(ids)

    # Dictionary that maps the index of temp vector to index in permanent vector
    indexes = {}

    ans = np.zeros((3*(l+1)))>3
    ind = 3
    for id in ids:
        i = id2ind_mu[id]
        ans[ind:ind+3] = mu[i:i+3]
        indexes[ind] = i
        ind+=3
    assert (np.count_nonzero(ans == False) == 0)

    # array and map
    return ans, indexes

def mu_temp2perm(mu, mu_temp, indexes):
    for ind in indexes.keys():
        i = indexes[ind]
        mu[i:i+3] = mu_temp[ind:ind+3]
    return mu

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)

def robot2world(r, tag):
    [x0, y0, th0] = r
    [x1, y1, th1] = tag
    
    return np.array([x0 + y1*np.cos(th0) + x1*np.sin(th0), -(-y0 + x1*np.cos(th0) - y1*np.sin(th0)), th0 + th1])

def euler_from_quaternion(x, y, z, w):

    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

def prediction(mu, sig, u, dt):
    W = np.random.normal(0, 0.1, sig.shape)

    mu_pred = mu + dt*u
    sig_pred = sig + W
    return mu_pred, sig_pred

def update(z, h, mu_pred, sig_pred, v):
    k = kalman_gain(sig_pred, h, v)

    mu_up = mu_pred + np.matmul(k, (z - np.matmul(h,mu_pred)))

    temp = np.matmul(k,h)
    i = np.eye(temp.shape[0])

    sig_up = np.matmul(i - temp,sig_pred)

    return mu_up, sig_up

def kalman_gain(sig_pred, h, v):
    ht = np.transpose(h)
    temp = np.linalg.inv(np.matmul(np.matmul(h, sig_pred),ht) + v)
    return np.matmul(np.matmul(sig_pred, ht),temp)

def tags_pose(camera_cmd):
    '''
    Tag pose in optical frame
    '''
    global mu
    global sigma
    global flag
    global traj

    # print('ja rha hai')
    state = mu[:3]
    print('state shape', state.shape)
    print('state', state)
    # z = np.array([None])
    ids = []
    # if len(camera_cmd.detections)>0:
    for tag in camera_cmd.detections:
        id = int(tag.id)
        ind = id2ind_mu[id]
        x1 = tag.pose.position.z
        y1 = -tag.pose.position.x
        qx = tag.pose.orientation.x
        qy = tag.pose.orientation.y
        qz = tag.pose.orientation.z
        qw = tag.pose.orientation.w
        o_quaternion = np.array([qx, qy, qz, qw])
        conversion = np.array([0.5, -0.5, 0.5, 0.5])
        r_quaternion = quaternion_multiply(conversion, o_quaternion)
        # y_about_x, z_about_y, x_about_z = euler_from_quaternion(*r_quaternion)
        y_about_x, z_about_y, x_about_z = euler_from_quaternion(qx, qy, qz, qw)

        # Tag state in robot frame
        print(real)
        real = r2w_w2r(mu[2])
        persp = np.array([x1, y1, z_about_y]).reshape(3,)
        # Rotated
        s = np.matmul(real,persp).reshape(3,)

        # Tag state in world frame
        sw = robot2world(state, s)
        ids.append(id)

        mu_ins[ind:ind+3] = sw
        z[ind:ind+3] = s

    n_tags = len(ids)
    mu_ins[:3] = state
    if n_tags > 0:
        h = h_mat(n_tags)
        v = np.eye(h.shape[0])

        sigma_prior, sig_indexes = temp_sig(sigma, ids)
        mu_prior, mu_indexes = temp_mu(mu_ins, ids)
        zz, z_indexes = temp_mu(z, ids)
        z_temp = zz[:3]

        mu_pred, sig_pred = prediction(mu_prior, sigma_prior, u, dt)
        mu_up, sig_up = update(z_temp, h, mu_pred, sig_pred, v)

        sigma = sig_temp2perm(sigma, sig_up, sig_indexes)
        mu = mu_temp2perm(mu, mu_up, mu_indexes)

    else:
        sigma_prior = sigma[:3,:3]
        mu_prior = state
        mu_pred, sig_pred = prediction(mu_prior, sigma_prior, u, dt)

        sigma[:3,:3] = sig_pred
        mu[:3] = mu_pred

    temp_pos = mu[:3]
    traj.append(temp_pos)
    pose.x = temp_pos[0]
    pose.y = temp_pos[1]
    pose.theta = temp_pos[2]
    print('updated_rbot_pos')
    print(pose)
    slam.publish(pose)

def twist_vel(twist_cmd):
    global u
    global dt
    u = np.array([twist_cmd.linear.z, twist_cmd.angular.x, twist_cmd.angular.y])
    dt = 0.1

if __name__ == "__main__":
    
    # Loop breaks when the robot is near the origin with below written tolerance
    while abs(x)+abs(y) > 0.1:
        # u, dt = 0, 0
        global u
        global dt
        u, dt = 0, 0

        rospy.init_node("slam")

        # for u and dt
        rospy.Subscriber('/twist', Twist, twist_vel, queue_size=1)
        # time before the observation
        time.sleep(dt)
        pose = Pose2D()
        # for z
        slam = rospy.Publisher("/slam", Pose2D, queue_size=1)
        # print('u, dt: ', u, dt)
        rospy.Subscriber('/apriltag_detection_array', AprilTagDetectionArray, tags_pose, queue_size=1)
        x, y = mu[0], mu[1]
        
    savetxt('trajectory.csv', traj, delimiter=',')
    savetxt('mu.csv', mu, delimiter=',')
    savetxt('sigma.csv', sigma, delimiter=',')
