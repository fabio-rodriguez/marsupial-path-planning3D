import matplotlib.pyplot as plt
import numpy as np

from pycatenary import cable
from tools import *
from bisect import bisect
from drawing import *



def get_min_catenary(top, T, obstacles, Lmin, Lmax, k_length, k_collision):    
    '''
    Take k_length lengths between the straight line and the L_max
    
    'min_cat_delta' evita que la libreria te de error con la primera catenaria por ser una recta
    '''

    for l in np.linspace(Lmin + EPSILON, Lmax, k_length):
        
        a = [0,0,0]
        f = T - top
        l1 = cable.MooringLine(L=l, w=0, EA=None, anchor=a, fairlead=f, floor=False)        
        l1.computeSolution()
        
        xyzs = []
        for s in np.linspace(0., l, k_collision):
            xyz = l1.s2xyz(s)
            xyzs.append(xyz)
            
        if xyzs[0][-1] < 0:
            xyzs += np.array([0,0,-xyzs[0][-1]]) + top
        else:
            xyzs += top


        for xyz in xyzs:
            if xyz[-1] <= 0:
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')
                ax.set_xlabel('X Axis')
                ax.set_ylabel('Y Axis')
                ax.set_zlabel('Z Axis')
                xx, yy, zz = zip(*xyzs)
                plt.title("ERROR")
                plt.plot(xx, yy, zz, '-b')   
                plt.show() 

                return -1
                
        is_collision_cat_obs = lambda oi: cat_obs_collision(oi, xyzs, k_collision)

        collision = False
        for oi in obstacles:    
            if is_collision_cat_obs(oi):
                for v in oi:
                    if v[-1] == 0:
                        return -1
                collision = True
                break                
            
        if collision:
            continue 

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')
        xx, yy, zz = zip(*xyzs)
        plt.plot(xx, yy, zz, '-b')   
        plt.show() 

        return l
    
    print("end")
    return -1

            

def cat_obs_collision(oi, xyzs, k_collision):

    # plane_number: 10 point_number: 10 t_first_step: 140.73789429664612 dij_time: 1.1343741416931152
    # plane_number: 10 point_number: 20 t_first_step: 274.8295590877533 dij_time: 1.9045300483703613

    if not len(oi):
        return False
    
    X_cat = list(list(zip(*xyzs))[0])
    is_reversed = False
    if X_cat[0] > X_cat[-1]:
        X_cat.reverse()
        is_reversed = True
    
    index = None
    for i in range(len(oi)):
        # if vertex_above_cat(oi[i], xyzs) != None:
        if vertex_above_cat2(X_cat, xyzs, is_reversed, oi[i]) != None:
            index = i
            break

    if index == None:
        return False

    # if vertex_above_cat(oi[index], xyzs):
    if vertex_above_cat2(X_cat, xyzs, is_reversed, oi[index]):
        for j in range(index+1, len(oi)):  # If above, search a point below
            # if vertex_above_cat(oi[j], xyzs) == False:
            if vertex_above_cat2(X_cat, xyzs, is_reversed, oi[j]) == False:
                return True
    else:
        oi = np.append(oi, [oi[0]], axis=0)        
        
        # result = False
        # import time
        # t = time.time()
        # for j in range(index+1, len(oi)-1):
        #     v1 = oi[j]
        #     v2 = oi[j+1]

        #     if v1[2] < 1 or v2[2] < 1:
        #         continue 
                    
        #     edge_length = euclidian_distance(v1, v2)
        #     edge_vector = normalize([v2-v1])[0]
        #     for step in numpy.linspace(0, 1, k_collision):  # Divided edge
        #         edge_point = v1 + edge_vector*step*edge_length 
        #         # if vertex_above_cat(edge_point, xyzs):  # Searching if edge above
        #         if vertex_above_cat2(X_cat, xyzs, is_reversed, edge_point):
        #             # return True
        #             result = True

        # print("top1", time.time()-t)
        # t = time.time()

        ##TOP2
        Z_cat = list(list(zip(*xyzs))[2])
        if is_reversed:
            Z_cat.reverse()

        Z_obs = np.zeros((len(X_cat),))
        for j in range(index+1, len(oi)-1):
            v1 = oi[j]
            v2 = oi[j+1]

            if v1[2] < 1 or v2[2] < 1:
                continue 
                
            xmin, zinit = min((v1[0], v1[2]), (v2[0], v2[2]))
            xmax, zend = max((v1[0], v1[2]), (v2[0], v2[2]))

            if xmax < X_cat[0] or xmin > X_cat[-1]:
                continue

            start_index = bisect(X_cat, xmin)
            end_index = bisect(X_cat, xmax)
            
            Xdist = xmax-xmin
            Zdist = zend-zinit
            for i in range(start_index, end_index):
                new_z = zinit + Zdist*(X_cat[start_index]-xmin)/Xdist 
                if new_z > Z_cat[i]:
                    return True
                
                if new_z > Z_obs[i]: 
                    Z_obs[i] = new_z
                else:
                    break

    return False


def vertex_above_cat(vertex, cat):

    vx, _, vz = vertex
    for i in range(len(cat)-1):
        x1, _, z1 = cat[i-1]
        x2, _, z2 = cat[i]
        lX, rX = min(x1, x2), max(x1, x2)

        if (lX <= vx <= rX):            
            return vz > z1 and vz > z2
    
    
def vertex_above_cat2(X_cat, cat, is_reversed, v):
    index = bisect(X_cat, v[0])
    if is_reversed:
        index = len(X_cat) - index - 1
    
    if 0 < index < len(X_cat)-1:
        return v[2] > cat[index][2] and v[2] > cat[index+1][2]
    else:
        return None



