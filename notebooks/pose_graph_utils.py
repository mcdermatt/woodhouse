from vedo import *
# from utils import *
import os
from ipyvtklink.viewer import ViewInteractiveWidget
import vtk
import numpy as np
import tensorflow as tf
from tensorflow.math import sin, cos, tan
# import tensorflow_probability as tfp
# import pickle
import matplotlib.pyplot as plt

# #limit GPU memory ------------------------------------------------
# gpus = tf.config.experimental.list_physical_devices('GPU')
# print(gpus)
# if gpus:
#   try:
#     memlim = 4*1024
#     tf.config.experimental.set_virtual_device_configuration(gpus[0], [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=memlim)])
#   except RuntimeError as e:
#     print(e)
# #-----------------------------------------------------------------


def R_tf(angs):
    """generates rotation matrix using euler angles
    angs = tf.constant(phi, theta, psi) (aka rot about (x,y,z))
            can be single set of angles or batch for multiple cells
    """

    if len(tf.shape(angs)) == 1:
        angs = angs[None,:]
    phi = angs[:,0]
    theta = angs[:,1]
    psi = angs[:,2]

    mat = tf.Variable([[cos(theta)*cos(psi), sin(psi)*cos(phi) + sin(phi)*sin(theta)*cos(psi), sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi)],
                       [-sin(psi)*cos(theta), cos(phi)*cos(psi) - sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(psi) + sin(theta)*sin(psi)*cos(phi)],
                       [sin(theta), -sin(phi)*cos(theta), cos(phi)*cos(theta)]
                        ])

    mat = tf.transpose(mat, [2, 0, 1])
    mat = tf.squeeze(mat)
    return mat

def v2t(vector):
    """converts a transformation vector to homogenous coordinate system"""
    if len(tf.shape(vector)) == 1: #allow for 1-D or N-D input
        vector = vector[None,:]
    angs = vector[:,3:]
    phi = angs[:,0]
    theta = angs[:,1]
    psi = angs[:,2]
    rot = tf.Variable([[cos(theta)*cos(psi), sin(psi)*cos(phi) + sin(phi)*sin(theta)*cos(psi), sin(phi)*sin(psi) - sin(theta)*cos(phi)*cos(psi)],
                       [-sin(psi)*cos(theta), cos(phi)*cos(psi) - sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(psi) + sin(theta)*sin(psi)*cos(phi)],
                       [sin(theta), -sin(phi)*cos(theta), cos(phi)*cos(theta)]])
    rot = tf.transpose(rot, [2, 0, 1])
    trans = vector[:,:3]
    trans = np.reshape(trans, (np.shape(rot)[0], 3, 1))
    transform = tf.concat((rot, trans), axis = -1)
    extra = tf.tile(tf.constant([[[0., 0., 0., 1.]]], dtype = tf.double), (np.shape(rot)[0],1,1))
    transform = tf.concat((transform, extra), axis = -2)
    return transform

def t2v(mat):
    """converts transformation matrix to state vector"""
    if len( tf.shape(mat) ) == 2:
        mat = mat[None, :, :]
    R_sum = np.sqrt(( mat[:,0,0]**2 + mat[:,0,1]**2 + mat[:,1,2]**2 + mat[:,2,2]**2 ) / 2)
    phi = np.arctan2(-mat[:,1,2],mat[:,2,2])
    theta = np.arctan2(mat[:,0,2], R_sum)
    psi = np.arctan2(-mat[:,0,1], mat[:,0,0])
    angs = np.array([phi, theta, psi])
    vector = tf.concat((mat[:,:3,-1], angs.T), axis =1)
    return vector

def get_J(e, ij):
    """Forms sparse jacobian J, with entries A and B at indices i and j 
    J == [N, 6, 6*N], N = number of nodes
    e == [N, 6] error vectors
    ij == [N, 2] contains scan indices of each node ex: [scan2, scan5]
    """
    total_num_of_nodes = np.max(ij) + 1 #TODO: is this too big??
    if len(tf.shape(ij))< 2: #expand dimensions if only single pair passed in
        ij = ij[None,:]
    A_ij, B_ij = get_A_ij_B_ij(e)
    
    # Need to tile DIFFERENT AMOUNT depending on the index 
    #    TODO: move to batch operation?
    J = tf.zeros([0,6,total_num_of_nodes*6])
    for k in range(len(ij)):
        #TODO: add logic for when i and j are same value, when j = i + 1 ...
        leading = tf.tile(tf.zeros([6,6]), [1, ij[k,0] ]) #leading zeros before i
#         print("\n leading \n", leading)
        between = tf.tile(tf.zeros([6,6]), [1, ij[k,1] - ij[k,0] - 1 ]) #zeros between i and j
#         print("\n between: \n", between)
        ending  = tf.tile(tf.zeros([6,6]), [1, total_num_of_nodes - ij[k,1] - 1 ])
        J_ij = tf.concat([leading, A_ij[k], between, B_ij[k], ending], axis = 1)
        # print("\n J(", k, ") \n", J_ij)      #for debug  
        J = tf.concat((J, J_ij[None,:,:]), axis = 0)

    # # test - apply constraint within J??
    # J = J.numpy()
    # J[0] += np.append(np.eye(6),  np.zeros([6,6*len(ij)]), axis = 1)
    # J = tf.convert_to_tensor(J)

    return J

def get_information_matrix(pred_stds):
    """returns information matrix (omega) from ICET cov_estimates"""

#     #I think this is wrong ... 
#     pred_stds = tf.convert_to_tensor(pred_stds)[:,:,None] #convert to TF Tensor
#     cov = pred_stds @ tf.transpose(pred_stds, (0,2,1))    #convert predicted stds -> covariance matrix
#     info = tf.linalg.pinv(cov) #invert
    
    #debug - set to identity
    info = tf.tile(tf.eye(6)[None,:,:], [tf.shape(pred_stds)[0] , 1, 1])
    info = tf.cast(info, tf.double)

#     #debug - weigh rotations more heavily than translations
# #     m = tf.linalg.diag(tf.constant([1., 1., 1., 10., 10., 10.]))
#     # m = tf.linalg.diag(tf.constant([10., 10., 10., 1., 1., 1.])) #vice-versa
#     # m = tf.linalg.diag(tf.constant([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])) #smol values
#     m = tf.linalg.diag(1000*tf.constant([1., 1., 1., 1., 1., 1.])) #big values
#     info = tf.tile(m[None,:,:], [tf.shape(pred_stds)[0] , 1, 1])
#     info = tf.cast(info, tf.double)


    return info

def get_b(e, omega, J):
    """gets b matrix, using batched operations """

    b = tf.math.reduce_sum(tf.cast(tf.transpose(J, (0,2,1)), tf.double) @ omega @ e, axis = 0)
    # print("\n b: \n", tf.shape(b))

    # #debug: zero out initial conditions
    # b = b.numpy()
    # b[:6] = 0
    # b = tf.convert_to_tensor(b)

    return b

def get_ij(ij_raw):
    """generates ij matrix, which describes which nodes are connected to 
       each other through odometry constraints. 
       Removes skipped indices and shifts everything to start at 0"""
#     print("ij_raw: \n", ij_raw)    
    y, idx = tf.unique(tf.reshape(ij_raw, [-1]))
#     print("\n y: \n", y, "\n idx: \n", idx)    
    ij = tf.reshape(idx, [-1,2])
    return ij

def get_H(J, omega):
    """returns hessian H"""

    # print("\n J: \n", tf.shape(J))
    # print("\n omega: \n", tf.shape(omega))

    H_ij = tf.transpose(J, (0,2,1)) @ tf.cast(omega, tf.float32) @ J
    H = tf.math.reduce_sum(H_ij, axis = 0)

    #produces negative eigenvals if we don't fix first point in chain
    # print("\n H before constraint: \n", tf.shape(H))
    # print("\n eigval H before constraint:\n", tf.linalg.eigvals(H))

    constrain_11 = tf.pad(tf.eye(6), [[0,len(H)-6],[0,len(H)-6]]) #was this
    # constrain_11 = tf.pad(tf.eye(6), [[1,len(H)-7],[1,len(H)-7]]) #test
    # constrain_11 = tf.pad(len(J) * tf.eye(6), [[0,len(H)-6],[0,len(H)-6]]) #test
    # print("constrain_11: \n", constrain_11)
    H = H + constrain_11

    # print("\n eigval H after constraint:\n", tf.linalg.eigvals(H))
#     H = tf.convert_to_tensor(np.tril(H.numpy()).T + np.tril(H.numpy(),-1)) #force H to be symmetric

    # #debug: zero out H11
    # H = H.numpy()
    # H[:6,:6] = 0
    # H = tf.convert_to_tensor(H)

    return H

def get_X(x, ij):
    """given x, a list of global poses, this function returns 
       the relative transformations X, that describe the same relationships described by the constraints z
       x  -> global poses of each transform
       ij -> indices of first and second element of x being considered
       """

    #get transform of fist elements in each pair, ordered by how they appear in ij
    first_vec = tf.gather(x, ij[:,0])
    second_vec = tf.gather(x, ij[:,1])

    first_tensor = v2t(tf.cast(first_vec, tf.double))
    second_tensor = v2t(tf.cast(second_vec, tf.double))

    #represents pose of x in 2nd node relative to pose in 1st
    
    #Problem with this answer is that this sets first tensor w.r.t. the world axis(?)
    X = tf.linalg.pinv(first_tensor) @ second_tensor #was this
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # #debug: invert coordinates of X
    # X = X.numpy()
    # X[:,:3,3] = -X[:,:3,3]
    # X = tf.convert_to_tensor(X)
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



    return X

def get_A_ij_B_ij(e_ij):
    """calculates nonzero terms from the Jacobian of error function w.r.t. nodes i and j using TensorFlow
        e_ij == error function [x, y, z, phi, theta, psi]
        
        NOTE: this works with batch operations: error vectors passed in as tensor will result in
                corresponding output in the same order 
    """
    e_ij = tf.cast(e_ij, tf.float32)
    p_point = e_ij[:,:3]
    phi = e_ij[:,3][:,None]
    theta = e_ij[:,4][:,None]
    psi = e_ij[:,5][:,None]
    
    eyes = tf.tile(-tf.eye(3)[None,:,:], [tf.shape(p_point)[0] , 1, 1]) #was this
#     eyes = tf.tile(tf.eye(3)[None,:,:], [tf.shape(p_point)[0] , 1, 1]) #test
    
#     deriv of R() wrt phi
    dRdPhi = tf.Variable([[tf.zeros(len(phi), dtype = phi.dtype)[:,None], (-sin(psi)*sin(phi) + cos(phi)*sin(theta)*cos(psi)), (cos(phi)*sin(psi) + sin(theta)*sin(phi)*cos(psi))],
                       [tf.zeros(len(phi), dtype = phi.dtype)[:,None], (-sin(phi)*cos(psi) - cos(phi)*sin(theta)*sin(psi)), (cos(phi)*cos(psi) - sin(theta)*sin(psi)*sin(phi))], 
                       [tf.zeros(len(phi), dtype = phi.dtype)[:,None], (-cos(phi)*cos(theta)), (-sin(phi)*cos(theta))] ])[:,:,:,0]
    dRdPhi = tf.transpose(dRdPhi, (2,0,1))
    Jx = dRdPhi @ p_point[:,:,None]
    
    # (deriv of R() wrt theta).dot(p_point)
    dRdTheta = tf.Variable([[(-sin(theta)*cos(psi)), (cos(theta)*sin(phi)*cos(psi)), (-cos(theta)*cos(phi)*cos(psi))],
                               [(sin(psi)*sin(theta)), (-cos(theta)*sin(phi)*sin(psi)), (cos(theta)*sin(psi)*cos(phi))],
                               [(cos(theta)), (sin(phi)*sin(theta)), (-sin(theta)*cos(phi))] ])[:,:,:,0]
    dRdTheta = tf.transpose(dRdTheta, (2,0,1))
    Jy = dRdTheta @ p_point[:,:,None]

    # deriv of R() wrt psi
    dRdPsi = tf.Variable([[(-cos(theta)*sin(psi)), (cos(psi)*cos(phi) - sin(phi)*sin(theta)*sin(psi)), (cos(psi)*sin(phi) + sin(theta)*cos(phi)*sin(psi)) ],
                                       [(-cos(psi)*cos(theta)), (-sin(psi)*cos(phi) - sin(phi)*sin(theta)*cos(psi)), (-sin(phi)*sin(psi) + sin(theta)*cos(psi)*cos(phi))],
                                       [tf.zeros(len(phi), dtype = phi.dtype)[:,None],tf.zeros(len(phi), dtype = phi.dtype)[:,None],tf.zeros(len(phi), dtype = phi.dtype)[:,None]]])[:,:,:,0]
    dRdPsi = tf.transpose(dRdPsi, (2,0,1))
    Jz = dRdPsi @ p_point[:,:,None]
    
    top = tf.concat([eyes, Jx, Jy, Jz], axis = 2) #was this
    # top = tf.concat([eyes, -Jx, -Jy, -Jz], axis = 2) #test
    flipped = tf.transpose(tf.concat([Jx, Jy, Jz], axis = 2), (0,2,1))     #was this
    
    bottom = tf.concat([-flipped, -eyes], axis = 2) #works???
#     bottom = tf.concat([flipped, -eyes], axis = 2) #test

    # #DEBUG: zero out non-diags
    # top = tf.concat([eyes, tf.zeros(tf.shape(flipped))], axis = 2) #test
    # bottom = tf.concat([tf.zeros(tf.shape(flipped)), -eyes], axis = 2) #test
    
    A_ij = tf.concat([top, bottom], axis = 1) #was this
    B_ij = -A_ij #was this
#     print("\n A_ij: \n", A_ij[0])
    return A_ij, B_ij

def get_e(Zij, Xij):
    """calculates error function w.r.t. nodes i and j
    Zij == pose j relative to i according to nodes (rotation matrix)
    Xij == pose j relative to i according to constraints (rotation matrix)
    """        

    # print("\n Xij \n", tf.shape(Xij))

    # was this ~~~~~~~~~~~~~~~~~~~~~~~

    # #need to set x -> x + delta_x
    e = t2v(tf.linalg.pinv(Zij) @ Xij) #translation right, rotation wrong
    # e = t2v(Zij @ Xij) #rotation right, translation wrong

    # #need to set x -> x - delta_x
    # # e = t2v(tf.linalg.pinv(Xij) @ Zij) 

    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 

    # e_trans = t2v(tf.linalg.pinv(Zij) @ Xij)[:,:3]
    # # e_trans = t2v(Xij)[:,:3] - t2v(Zij)[:,:3] #test
    # # e_trans = -t2v(tf.transpose(Xij, (0,2,1)) @ Zij)[:,:3] #test
    # print("\n e_trans: \n", e_trans[:5])

    # e_rot = t2v(Zij @ Xij)[:,3:]
    # print("\n e_rot: \n", e_rot[:5])
    # # e = tf.concat((e_trans, e_rot), axis = 1)
    print("\n e: \n", e)    

    # e = t2v(Xij) - t2v(Zij) #debug
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 

    # print("\n Xij vec: \n", t2v(Xij))
    # print("\n Zij vec: \n", t2v(Zij))
    
    return(e)    

def R2Euler(mat):
    """determines euler angles from euler rotation matrix"""

    if len( tf.shape(mat) ) == 2:
        mat = mat[None, :, :]

    R_sum = np.sqrt(( mat[:,0,0]**2 + mat[:,0,1]**2 + mat[:,1,2]**2 + mat[:,2,2]**2 ) / 2)

    phi = np.arctan2(-mat[:,1,2],mat[:,2,2])
    theta = np.arctan2(mat[:,0,2], R_sum)
    psi = np.arctan2(-mat[:,0,1], mat[:,0,0])

    angs = np.array([phi, theta, psi])
    return angs

class Ell(Mesh):
    """
    Build a 3D ellipsoid centered at position `pos`.

    |projectsphere|

    |pca| |pca.py|_
    """
    def __init__(self, pos=(0, 0, 0), axis1= 1, axis2 = 2, axis3 = 3, angs = np.array([0,0,0]),
                 c="cyan4", alpha=1, res=24):

        self.center = pos
        self.va_error = 0
        self.vb_error = 0
        self.vc_error = 0
        self.axis1 = axis1
        self.axis2 = axis2
        self.axis3 = axis3
        self.nr_of_points = 1 # used by pcaEllipsoid

        # if utils.isSequence(res): #new Vedo
        if utils.is_sequence(res): #old Vedo
            res_t, res_phi = res
        else:
            res_t, res_phi = 2*res, res

        elliSource = vtk.vtkSphereSource()
        elliSource.SetThetaResolution(res_t)
        elliSource.SetPhiResolution(res_phi)
        elliSource.Update()
        l1 = axis1
        l2 = axis2
        l3 = axis3
        self.va = l1
        self.vb = l2
        self.vc = l3
        axis1 = 1
        axis2 = 1
        axis3 = 1
        angle = angs[0] #np.arcsin(np.dot(axis1, axis2))
        theta = angs[1] #np.arccos(axis3[2])
        phi =  angs[2] #np.arctan2(axis3[1], axis3[0])

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Scale(l1, l2, l3)

        #needed theta and angle to be negative before messing with E_xz, E_yz...
        t.RotateZ(np.rad2deg(phi))
        t.RotateY(-np.rad2deg(theta)) #flipped sign here 5/19
        t.RotateX(-np.rad2deg(angle)) #flipped sign here 5/19
        
        tf = vtk.vtkTransformPolyDataFilter()
        tf.SetInputData(elliSource.GetOutput())
        tf.SetTransform(t)
        tf.Update()
        pd = tf.GetOutput()
        self.transformation = t

        Mesh.__init__(self, pd, c, alpha)
        self.phong()
        self.GetProperty().BackfaceCullingOn()
        self.SetPosition(pos)
        self.Length = -np.array(axis1) / 2 + pos
        self.top = np.array(axis1) / 2 + pos
        self.name = "Ell"

def draw_body_frame_axis(x, disp = []):
    """draws local xyz axis arrows on each pose in x """
    scale = 0.25 #axis length
    alpha = 1
#     print(x)
    rot = x[:3,:3]
    A = rot @ np.eye(3) * scale
    xvec = Arrow(x[:3,-1], x[:3,-1] + A[:,0], c = 'red', alpha = alpha) 
    yvec = Arrow(x[:3,-1], x[:3,-1] + A[:,1], c = 'green', alpha = alpha) 
    zvec = Arrow(x[:3,-1], x[:3,-1] + A[:,2], c = 'blue', alpha = alpha) 
    disp.append(xvec)
    disp.append(yvec)
    disp.append(zvec)
    return(disp)

def plot_results(disp, result, ij, marginals = None, draw_axis = False):
    
    #plot coordinates of point centers
    for i in range(1,result.size()): #loop through all elements in results
        # print(i)
        p = result.atPose3(i).matrix()
        p_center = Points([[p[0,-1], p[1,-1],  p[2,-1]]], r = 5)
        disp.append(p_center)
        #add text labels
        # t = Text3D(i, p[:3,-1], s = 0.01, alpha = 0.3).followCamera() #need for deskop
        t = Text3D(i, p[:3,-1], s = 0.01, alpha = 0.3).follow_camera() #newer Vedo on Laptop?
        disp.append(t)
        if draw_axis is True:
            disp = draw_body_frame_axis(p, disp) #draw coordinate frames
        if marginals and i < len(ij): #num marginals in a chain = length of chain -1
            frame = result.atPose3(ij[i,0]).matrix()#[None,:]
            sigma = marginals.marginalCovariance(i)[:3,:3] #[None,:3,:3] #just ignore parts of sigma related to rotations
            sigma = tf.convert_to_tensor(sigma)
            disp = draw_ell(disp, frame, sigma)
            #TODO: plot last ellispe
            
    #draw constraints using ij
    for c in range(1,len(ij)):
        pt1 = result.atPose3(ij[c,0]).translation()   #get coords of results i and j 
        pt2 = result.atPose3(ij[c,1]).translation()
        L = Line(p0 = pt1, p1 = pt2, lw = 1)
        disp.append(L)

    return(disp)

def draw_ell(disp, frame, sigma, pc = 1, alpha = 0.5):
    """draw distribution ellipses given mu and sigma tensors"""
    color = [0.3, 0.3, 0.8]

    mu = frame[:3,-1]
    eig = np.linalg.eig(sigma[:,:].numpy())
    eigenval = eig[0] #correspond to lengths of axis
    eigenvec = eig[1]

    #eigenvals not ordered in decreasing size when scale large enough
    a1 = tf.sort(eigenval).numpy()[-1]
    a2 = tf.sort(eigenval).numpy()[-2]
    a3 = tf.sort(eigenval).numpy()[-3]
#     a1 = eigenval[0]
#     a2 = eigenval[1]
#     a3 = eigenval[2]
    
    #only need 3x3 of eigenvector to describe rotation of covariance ellipse
    eigenvec = tf.gather(eigenvec, tf.argsort(eigenval, direction='DESCENDING'), axis = 1)
#     eigenvec = tf.gather(eigenvec, tf.argsort(eigenval, direction='DESCENDING'), axis = 0)

    #rotate into frame of previous odometry estimate (instead of world xyz frame)     
#         eigenvec = eigenvec @ frame[0,:3,:3]  #frame has axis switched up in some frames
#         eigenvec = eigenvec @ tf.gather(frame[0,:3,:3], tf.argsort(eigenval, direction='DESCENDING'), axis = 1)      
    #DEBUG- issues because each frame is not centered w.r.t origin??
    rot = frame[:3,:3]
#         rot = -tf.linalg.pinv(frame[:3,:3])
#     print("\n rot \n:",  rot)
#     print("\n eigenvec before: \n", eigenvec)
    eigenvec = eigenvec @ rot #works(ish) if I comment this out ...
#         print("\n eigenvec after: \n", eigenvec)

#     #DEBUG--------------------------
#     scale = 2
#     rot = frame[:3,:3]
# #         E = np.eye(3) #TODO: need to switch order of rows with a1,a2,a3
#     correct_order = tf.argsort(np.linalg.eig(sigma[:,:].numpy())[0], direction = "DESCENDING").numpy()
# #         correct_order = tf.argsort(np.linalg.eig(sigma[:,:].numpy())[0]).numpy()
# #     E = np.eye(3)[correct_order]
#     E = np.eye(3)
#     A = rot @ E * scale * np.sqrt(np.array([a1, a2, a3]))
#     xvec = Arrow(frame[:3,-1], frame[:3,-1] + A[:,0], c = 'red', alpha = alpha) 
#     yvec = Arrow(frame[:3,-1], frame[:3,-1] + A[:,1], c = 'green', alpha = alpha) 
#     zvec = Arrow(frame[:3,-1], frame[:3,-1] + A[:,2], c = 'blue', alpha = alpha) 
#     disp.append(xvec)
#     disp.append(yvec)
#     disp.append(zvec)
#     # -------------------------------

    if mu[0] != 0 and mu[1] != 0:
        ell = Ell(pos=(mu[0], mu[1], mu[2]), axis1 = 4*np.sqrt(abs(a1)), 
        axis2 = 4*np.sqrt(abs(a2)), axis3 = 4*np.sqrt(abs(a3)), 
#         angs = (np.array([R2Euler(rot)[0], R2Euler(rot)[1], R2Euler(rot)[2] ])), c=color, alpha=alpha, res=12)
        angs = (np.array([R2Euler(eigenvec)[0], R2Euler(eigenvec)[1], R2Euler(eigenvec)[2] ])), c=color, alpha=alpha, res=12)
        disp.append(ell)

    return(disp)