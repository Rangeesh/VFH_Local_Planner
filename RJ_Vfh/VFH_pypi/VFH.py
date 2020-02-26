import open3d as o3d
import numpy as np
def process_feat(feat):
    ind_nan = np.where(np.isnan(feat))
    ind_inf = np.where(np.isinf(feat))
    feat[ind_inf] = np.nan
    inds = np.where(np.isnan(feat))
    row_mean = np.nanmean(feat)
    feat[inds] = np.take(row_mean, inds[0])
    return feat
def VFH(pcd):
    if(pcd.__class__.__name__=="ndarray"):
        pointcloud = o3d.geometry.PointCloud()
        pointcloud.points = o3d.utility.Vector3dVector(pcd)
        pointcloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1, max_nn=30))
        points=np.asarray(pcd)
        normals = np.asarray(pointcloud.normals)
    elif(pcd.__class__.__name__=="PointCloud"):
        pointcloud=pcd
        points = np.asarray(pointcloud.points)
        pointcloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1, max_nn=30))
        normals = np.asarray(pointcloud.normals)

    p_c = pointcloud.get_center()
    n_c = sum(np.asarray(pointcloud.normals))/len(np.asarray(pointcloud.normals))
    p_v = np.asarray([0.0, 0.0, 0.0])
    
    u = n_c
    tmp = np.asarray([(pi - p_c)/np.linalg.norm(pi - p_c) for pi in list(pointcloud.points)])
    v = np.asarray([np.cross(t, u) for t in list(tmp)])
    w = np.asarray([np.cross(u, t) for t in list(v)])

    cos_a = []
    alpha = []
    for i, j in zip(v, normals):
        dot = np.dot(i, j)
        cos_a.append(dot)
        alpha.append(round(np.arccos(dot) * (180 / np.pi)))
    
    alpha = np.asarray(alpha)
    cos_b = []
    beta = []
    for n in normals:
        tmp = (p_c - p_v) / np.linalg.norm(p_c - p_v)
        dot = np.dot(n, tmp)
        cos_b.append(dot)
        beta.append(round(np.arccos(dot) * (180 / np.pi)))
    beta = np.asarray(beta)

    d = np.asarray([ np.linalg.norm(pi - p_c) for pi in points])


    cos_phi = []
    phi = []
    for i, j in zip(points, d):
        tmp = (i - p_c) / j
        dot = np.dot(u, tmp)
        cos_phi.append(dot)
        phi.append(round(np.arccos(dot) * (180 / np.pi)))
    
    phi = np.asarray(phi)

    theta = []
    for i, j in zip(w, normals):
        tmp = (np.dot(i, j)) / (np.dot(u, j))
        theta.append(round(np.arctan(tmp) * (180 / np.pi)))
    theta = np.asarray(theta)
    
    his_alpha = np.histogram(cos_a, bins = 45, density = True)
    his_phi =  np.histogram(cos_phi, bins = 45, density = True)
    his_theta = np.histogram(theta, bins = 45, density = True)
    his_d = np.histogram(d, bins = 45, density = True)
    his_beta = np.histogram(cos_b, bins = 128, density = True)
    com_his = np.concatenate((his_alpha[0], his_phi[0], his_theta[0], his_d[0], his_beta[0]), axis = 0)
    com_his=process_feat(com_his)
    return com_his
    