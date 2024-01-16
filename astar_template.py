import numpy as np
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, find, get_closest_edge_point, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
import time

### YOUR IMPORTS HERE ###
from utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, wait_if_gui, wait_for_user, joint_from_name, get_joint_info, get_link_pose, link_from_name
from queue import PriorityQueue
from utils_astar import Node, getG, getH, ComputeF, reachGoal

#########################

def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('pr2doorway.json')

    # define active DoFs
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

    # Example use of setting body poses
    # set_pose(obstacles['ikeatable6'], ((0, 0, 0), (1, 0, 0, 0)))
    
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints))
    goal_config = (2.6, -1.3, -np.pi/2) # x, y, theta
    # goal_config = (2.6, 1, -np.pi/2) # x, y, theta
    path = []
    start_time = time.time()
    ### YOUR CODE HERE ###
 
    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    def collision(x, y, yaw):
        return collision_fn((x, y, yaw))

    PathCost = -1

    TEST_LENGTH = 5000

    GOAL_THRESHOLD = 0.1 # So far best working
    STEP_SIZE = 0.15 # Old: 0.2 - Same steps for both x and y

    q = PriorityQueue()
    start_x, start_y, start_t = start_config
    start_g = 0 # cost of start node
    gx, gy, gt = goal_config

    ### Draw start and goal pose
    sphere_radius = 0.1
    sphere_color_r = (1, 0, 0, 1) # R, G, B, A 
    print("================================================")
    print("Start: ", start_config)
    draw_sphere_marker((start_x, start_y, 0), sphere_radius, sphere_color_r)
    print("Goal: ", goal_config)
    sphere_color_g = (0, 1, 0, 1) # R, G, B, A 
    draw_sphere_marker((gx, gy, 0), sphere_radius, sphere_color_g)


    def node2id(node):
        return str(node.x) + str(node.y)+str(node.theta)
    id = 0
    start = Node(start_x, start_y, start_t, start_g)
    priority = ComputeF(start_x,start_y,start_t, start_g, start_x, start_y, start_t, gx, gy, gt)
    q.put( (priority, id, start)  )

    # Visualization
    Astar_path_circles = [] 
    Astar_obs_nodes = []  
    Astar_free_nodes = [] 

    open = []
    closed = []
    Parent = {}
    gcost = {}
    fcost = {}
    root_parent = Node(0,0,0,0)
    Parent[start] = root_parent 
    open.append(node2id(start)) 
    
    def ExtractPath(node):
        while Parent[node] !=  root_parent:
            # path.append((node.x, node.y, 0.3)) # z=0
            path.append((node.x, node.y, node.theta))
            Astar_path_circles.append((node.x, node.y, 0.3))
            node = Parent[node]
        path.reverse()


    directions4 = [(1,0), (-1,0), (0,1), (0,-1)]
    directions8 = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1),  (1,-1), (-1,-1)]
    
    #degrees = [0, np.pi/2, np.pi, -np.pi/2] #TODO : Theta shouldn't be taken according to Piazza Post by Abhinav!!!
    degrees = [gt]
    iter = 0
    findpath = False

    FACTOR = 500
    FOUR_CONNECTED = False # TODO: Final submission requires False : EIGHT_NEIGHBOURS
    DRAW_EXPLORATION_NODES = True
    
    if FOUR_CONNECTED:
        directions = directions4
        print("4-CONNECTED NEIGHBORS")
    else:
        directions = directions8
        print("8-CONNECTED NEIGHBORS")

    while not q.empty() and iter<TEST_LENGTH:
        iter += 1
        if iter%FACTOR==0:
            print("iter: ",iter)
        curNode = q.get()

        nx, ny, nt, ng = curNode[2].getXYTg()

        # if reached goal within certain threshold
        if reachGoal(nx, ny, nt, gx, gy, gt, GOAL_THRESHOLD):
            print(f"Reached Goal at iteration = {iter}")
            ExtractPath(curNode[2])
            PathCost = curNode[0] # Its priority
            print("Final Pose: (x,y,theta): ",round(nx,4), round(ny,4), round(nt,4))
            findpath = True
            break

        # Place current node from openlist to closedlist
        open.remove(node2id(curNode[2]))
        # closed.append(Node(nx, ny, nt, ng))
        closed.append(node2id(curNode[2]))

        for dir in directions:
            dx, dy = dir
            mx = nx + dx*STEP_SIZE
            my = ny + dy*STEP_SIZE
            for orientation in degrees:
                mt = orientation
                mg = ng + getG(nx, ny, nt, mx, my, mt)
                id = id+1
                nextNode = Node(mx, my, mt, mg)
                nextNodeid = node2id(nextNode)
                # mt = nt ### TODO : Theta shouldn't be taken according to Piazza Post!!!
                if  nextNodeid not in closed and not collision(mx, my, mt): # visit and add them to q (open set)
                    Astar_free_nodes.append((mx,my,0))  
                    priority = ComputeF(nx, ny, nt, ng, mx, my, mt, gx, gy, gt)
                    new_g = ng + getG(nx, ny, nt, mx, my, mt)
                    change_parent = False

                    #q.put( (priority, id, nextNode))
                    if nextNodeid not in open:
                        change_parent = True
                        q.put( (priority, id, nextNode)) 
                        open.append(nextNodeid)                   
                    elif new_g < gcost[nextNodeid]:
                        change_parent = True
                    if change_parent:
                        Parent[nextNode] = curNode[2]
                        gcost[nextNodeid] = new_g
                        fcost[nextNodeid] = ComputeF(nx, ny, nt, ng, mx, my, mt, gx, gy, gt)

                elif nextNodeid not in closed and collision(mx,my,mt):
                    Astar_obs_nodes.append((mx,my,0))

        # print("Priority:", next_item[0])
        # next_item[2].printme()

    
    if not findpath:
        print("No path with this start and goal configuratoin\n")

    final_time = time.time() - start_time
    print("Execution Time: ", round(final_time,4)) 
    print("Path cost: ", round(PathCost,4))


    ############### Draw the path ################
    
    sphere_radius = 0.1
    sphere_color_black = (0, 0, 0, 1) # R, G, B, A
    # for pose in path:
    Astar_path_circles.reverse()
    for pose in Astar_path_circles:
        draw_sphere_marker(pose, sphere_radius, sphere_color_black)

    print("=======================================")
    print("Now executing the path:")
    #print("path: ",path)
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)

    ######################

    Astar_free_nodes = list(set(Astar_free_nodes))
    Astar_obs_nodes = list(set(Astar_obs_nodes))

    print("# Free space nodes explored: ",len(Astar_free_nodes))
    print("# Obstacle nodes encountered: ",len(Astar_obs_nodes))


    if DRAW_EXPLORATION_NODES:
        print("Plotting free space node...")
        sphere_color_blue = (0, 0, 1, 1)
        for pose in Astar_free_nodes:
            draw_sphere_marker(pose, sphere_radius, sphere_color_blue)
        
        print("Plotting obstacle nodes: ...")
        for pose in Astar_obs_nodes:
            draw_sphere_marker(pose, sphere_radius, sphere_color_r)        
        print("=======================================")

    # ######################
    # Execute planned path
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()