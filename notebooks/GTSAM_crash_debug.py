import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
# from ipyvtklink.viewer import ViewInteractiveWidget
from vedo import  *
import tensorflow as tf
from tensorflow import  sin, cos, tan
from pose_graph_utils import *

gpus = tf.config.experimental.list_physical_devices('GPU')
print(gpus)

npts = 20
odometry_history = np.tile(np.array([0.0, 0.15, 0.08, 0.0, 0.0, 0.3]), (npts,1)) #will create corkscrew motion
# #add process noise
# odometry_history += np.random.randn(np.shape(odometry_history)[0], 
#                                     np.shape(odometry_history)[1])*np.array([0,0,0, 0.0,0.1,0.])  
# make up history of standard deviations of measurement uncertainty
pred_stds_history = np.tile(np.array([[0.001, 0.005, 0.001, 1e-6, 1e-6, 1e-5]]), (len(odometry_history),1))
ij = np.array([[0,1]])
for i in range(1,len(odometry_history)):
    ij = np.append(ij, np.array([[i, i+1]]), axis = 0)

odometry_history = np.append(odometry_history, ij, axis= 1)   #match format output by ROS package
pred_stds_history = np.append(pred_stds_history, ij, axis= 1) #match format output by ROS package

# Create an empty nonlinear factor graph
graph = gtsam.NonlinearFactorGraph() 

# Add a prior on the first pose, setting it to the origin
# A prior factor consists of a mean and a noise model (covariance matrix)
# priorMean = gtsam.Pose3(np.zeros([6,1], dtype = np.float64))  # prior at origin

print("\n made it here \n ")

test = tf.constant([1.,0.1])
print(test)

test2  = R_tf(np.array([0.,0.,0.]))
print(test2, type(test2))

priorRot = gtsam.Rot3(R_tf(np.array([0., 0., 0.8]).numpy()))
# priorRot = gtsam.Rot3(R.from_euler('xyz',[0., 0., 0.8]).as_matrix())

print("made it past the first fail")

priorMean = gtsam.Pose3(priorRot, np.array([0., 0., .0])) #prior at nonzero pose
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.003, 0.003, 0.003, 1e-5, 1e-5, 1e-5], dtype = np.float64))
# firstKey = initial.keys()[0] #TODO: what does this do???

graph.add(gtsam.PriorFactorPose3(0, priorMean, PRIOR_NOISE)) #constrain first point at priorMean

#loop through all constraints 
for i in range(len(ij)):
# for i in range(int(np.max(ij))):
# for i in range(1,int(np.max(ij)) + 1): #test
    #get constraint 
    raw_odom = odometry_history[i,:-2] #omit scan indices stored at end of line
    #convert to Point3 strucutre
    point = raw_odom[:3] 
    rot = gtsam.Rot3(R_tf(raw_odom[3:]))
    odometry_estimate = gtsam.Pose3(rot, point)
#     print("\n odometry estimate:\n", odometry_estimate)
    cov_estimate = gtsam.noiseModel.Diagonal.Sigmas(pred_stds_history[i,:-2])
#     cov_estimate = gtsam.noiseModel.Diagonal.Sigmas(pred_stds_history[i,:]) #dont need to omit scan indices for fake data
    first_idx = ij[i,0]
    second_idx = ij[i,1] 
#     print(first_idx, second_idx) #misses last element in ij!!!
    graph.add(gtsam.BetweenFactorPose3(first_idx, second_idx, odometry_estimate, cov_estimate))

# print(graph.size())
#set to zero initial conditions
initial = gtsam.Values()
print(initial.keys())
print("\n graph size:", graph.size())
# print(int(np.max(ij)))
# for j in range(graph.size()):
for j in range(int(np.max(ij))+1):
#     zero_rot = gtsam.Rot3(R_tf(np.array([0., 0., 0.])))
#     zero_pose3 = gtsam.Pose3(zero_rot, np.array([0., 0., 0.])) #does not work with zero initial conds??
#     initial.insert(j, zero_pose3)
    init_rot = gtsam.Rot3(R_tf(0.01*np.random.randn(3)))
    init_pose = gtsam.Pose3(init_rot, np.random.randn(3))
    initial.insert(j, init_pose)
    
# optimize using Levenberg-Marquardt optimization
# damped optimizer - seems to work much better here
params = gtsam.LevenbergMarquardtParams()
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)

# # simple gauss newton - kinda unreliable for high DOF problems (especially with zero initial conditions)
# params = gtsam.GaussNewtonParams()
# params.setVerbosity("Termination")  # this will show info about stopping conds
# optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)

# # dogleg
# params = gtsam.DoglegParams()
# params.setVerbosity("Termination")
# params.setMaxIterations(100)
# optimizer = gtsam.DoglegOptimizer(graph, initial, params)

result = optimizer.optimize()
marginals = gtsam.Marginals(graph, result) #calculate covariance estimates for each pose

print("initial error = ", graph.error(initial))
print("final error = ", graph.error(result))
# print(initial.keys())
# print(graph.keys())