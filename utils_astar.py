import numpy as np

class Node:
    def __init__(self, x_in,y_in,theta_in, g):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.g = g

    def printme(self):
        print("\tNode:", "x =", self.x, "y =",self.y, "theta =", self.theta,  "g =",self.g)
    def getXYTg(self):
        return self.x, self.y, self.theta, self.g

def getG(nx,ny, nt, mx, my, mt):
    g = pow( (pow((mx-nx),2) + pow((my - ny),2) + min(abs(mt-nt),  2*np.pi-abs(mt-nt))), 0.5  )
    return g

def getH(x,y, theta, gx, gy, gtheta):
    h  = pow( (pow((x-gx),2) + pow((y - gy),2) + min(abs(theta-gtheta),  2*np.pi-abs(theta-gtheta))), 0.5  )
    return h

def ComputeF(lastx, lasty, lastt, lastg, x, y, t, gx, gy, gt):
    g = lastg  + getG(lastx, lasty, lastt, x, y, t)
    h = getH(x,y,t, gx, gy, gt)
    f = g + h
    return f

def reachGoal(x, y, t, gx, gy, gt, goal_threshold):
    if getH(x,y, t, gx, gy, gt) <= goal_threshold:
        return True
    else:
        return False

# def performAstar(goal_config, ):
#     while not q.empty() and iter<TEST_LENGTH:
#         iter += 1
#         if iter%FACTOR==0:
#             print("iter: ",iter)
#         curNode = q.get()

#         nx, ny, nt, ng = curNode[2].getXYTg()

#         # if reached goal within certain threshold
#         if reachGoal(nx, ny, nt, gx, gy, gt, GOAL_THRESHOLD):
#             print(f"Reached Goal at iteration = {iter}")
#             ExtractPath(curNode[2])
#             PathCost = curNode[0] # Its priority
#             print("Final Pose: (x,y,theta): ",round(nx,4), round(ny,4), round(nt,4))
#             findpath = True
#             break

#         # Place current node from openlist to closedlist
#         open.remove(node2id(curNode[2]))
#         # closed.append(Node(nx, ny, nt, ng))
#         closed.append(node2id(curNode[2]))

#         for dir in directions:
#             dx, dy = dir
#             mx = nx + dx*STEP_SIZE
#             my = ny + dy*STEP_SIZE
#             for orientation in degrees:
#                 mt = orientation
#                 mg = ng + getG(nx, ny, nt, mx, my, mt)
#                 id = id+1
#                 nextNode = Node(mx, my, mt, mg)
#                 nextNodeid = node2id(nextNode)
#                 # mt = nt ### TODO : Theta shouldn't be taken according to Piazza Post!!!
#                 if  nextNodeid not in closed and not collision(mx, my, mt): # visit and add them to q (open set)
#                     Astar_free_nodes.append((mx,my,0))  
#                     priority = ComputeF(nx, ny, nt, ng, mx, my, mt, gx, gy, gt)
#                     new_g = ng + getG(nx, ny, nt, mx, my, mt)
#                     change_parent = False

#                     #q.put( (priority, id, nextNode))
#                     if nextNodeid not in open:
#                         change_parent = True
#                         q.put( (priority, id, nextNode)) 
#                         open.append(nextNodeid)                   
#                     elif new_g < gcost[nextNodeid]:
#                         change_parent = True
#                     if change_parent:
#                         Parent[nextNode] = curNode[2]
#                         gcost[nextNodeid] = new_g
#                         fcost[nextNodeid] = ComputeF(nx, ny, nt, ng, mx, my, mt, gx, gy, gt)

#                 elif nextNodeid not in closed and collision(mx,my,mt):
#                     Astar_obs_nodes.append((mx,my,0))