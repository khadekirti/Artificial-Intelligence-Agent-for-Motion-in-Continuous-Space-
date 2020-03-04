'''
    Artificial Intelligence :  Assignment 2
    Vikram Rayakottai Niranjanvel- 45484450
    Kirti Khade- 45733130
 
'''
import os
from support.problem_spec import ProblemSpec
import numpy as np
import random
import sys
from example_search import solve, GraphNode
import math
import copy
from support.angle import Angle
from support.robot_config import make_robot_config_from_ee1, write_robot_config_list_to_file, RobotConfig,make_robot_config_from_ee2
import cmath
from tester import test_orientation, test_environment_bounds, test_angle_constraints, test_line_collision, \
    test_config_equality, test_length_constraints, test_self_collision, test_obstacle_collision, test_config_distance
from copy import deepcopy
from numpy import arange
import time

 

def get_intercetions(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1
    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    # non intersecting
    if d > r0 + r1:
        return None
    # One circle within other
    if d < abs(r0 - r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(abs(r0 ** 2 - a ** 2))
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d
        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d
        return ((x3, y3), (x4, y4))


def getPointsCircle(startingPoint, numberPoints, lenght):
    # Generate unit cirle with k points
    k = np.arange(numberPoints)
    # Get points for unit circle with (0,0) centre
    points = np.exp(2.0 * np.pi * 1j * k / numberPoints)
    # Get new points of lenght l
    points = lenght * points
    # Add the cordinates of the starting point
    points += startingPoint
    return points


def ComplexCordinate(z):
    return (z.real, z.imag)


def CordinateComplex(x, y):
    return x + y * 1j


# Distance between two points
def distancePoints(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def getAngleM(a, b):
    ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    return ang + 360 if ang < 0 else ang


def pointsToConfig(pointsConfig):
    angles = []
    length = []
    for i in range(0, len(pointsConfig) - 1):
        angles.append(
            getAngleM((pointsConfig[i][0], pointsConfig[i][1]), (pointsConfig[i + 1][0], pointsConfig[i + 1][1])))
        length.append(
            distancePoints((pointsConfig[i][0], pointsConfig[i][1]), (pointsConfig[i + 1][0], pointsConfig[i + 1][1])))

    for i in range(1, len(angles)):
        angles[i] -= sum(angles[0:i])


    anglesAngles = []
    for i in range(0, len(angles)):
        anglesAngles.append(Angle(degrees=angles[i]))

    # Making it from ee1
    config = make_robot_config_from_ee1(pointsConfig[0][0], pointsConfig[0][1], anglesAngles, length, ee1_grappled=True)

    return config


def valid_config(config, spec):
    """
    #Imported from tester
    #Self Collision
    #Bound check
    #Angle constraints
    #Length constraints
    #Obstacle collision
    """
    if test_self_collision(config, spec) == False:
        return False
    if test_environment_bounds(config) == False:
        return False
    if test_angle_constraints(config, spec) == False:
        return False
    if test_length_constraints(config, spec) == False:
        return False
    if test_obstacle_collision(config, spec, spec.obstacles) == False:
        return False
    return True


def points3rod(spec, min_len, max_len, initial, from_, goal):
    all_configs = []
    for r in arange(min_len[0], max_len[0] + 0.1, 0.1):
        # From the point as a centre, create a circle for 30 points
        pointsCircle = getPointsCircle(CordinateComplex(from_[0], from_[1]), 100, r)
        for z in pointsCircle:
            (x, y) = ComplexCordinate(z)
            # for the 2nd point
            for r2 in arange(min_len[1], max_len[1] + 0.1, 0.1):
                # for the third point
                for r3 in arange(min_len[2], max_len[2] + 0.1, 0.1):
                    # get the cordinate points
                    if get_intercetions(x, y, r2, goal[0], goal[1], r3):
                        a, b = get_intercetions(x, y, r2, goal[0], goal[1], r3)
                        for p in (a, b):
                            config = pointsToConfig((initial, from_, (x, y), p, goal))
                            if valid_config(config, spec):
                                return config
    return None


def config4Points(spec, initial, goal,min_len,max_len):
    looped = 0
    while (looped < 10):
        for r in arange(min_len[0], max_len[0] + 0.1, 0.1):
            looped += 1
            pointsCircle = getPointsCircle(CordinateComplex(initial[0], initial[1]), 100, r)
            m = random.choice(pointsCircle)
            (i, j) = ComplexCordinate(m)
            config = points3rod(spec, min_len[1:], max_len[1:], initial, (i, j), goal)
            if config is not None:
                return config
    return None

def config4Obstacle(points,spec,grapple_points,min_len,max_len):
    config_for_every_point = []
    for p in points:
        gp = grapple_points[0]
        config  = config4Points(spec,gp,p,min_len,max_len)
        if config is not None:
            config_for_every_point.append([config])
    return config_for_every_point
 
    

def config3Points(spec, initial, goal,min_len,max_len):
    all_config = []
    for r in arange(min_len[0], max_len[0] + 0.1, 0.1):
        # From the point as a centre, create a circle for 30 points
        pointsCircle = getPointsCircle(CordinateComplex(initial[0], initial[1]), 100, r)
        z = random.choice(pointsCircle)
        (x, y) = ComplexCordinate(z)
        # for the 2nd point
        for r2 in arange(min_len[1], max_len[1] + 0.1, 0.1):
            # for the third point
            for r3 in arange(min_len[2], max_len[2] + 0.1, 0.1):
                # get the cordinate points
                if get_intercetions(x, y, r2, goal[0], goal[1], r3):
                    a, b = get_intercetions(x, y, r2, goal[0], goal[1], r3)
                    for p in (a, b):
                        # Get  point to config
                        config = pointsToConfig((initial, (x, y), p, goal))
                        if valid_config(config, spec):
                            all_config.append(config)
    return all_config


def config3Obstacle(points, spec,grapple_points, min_len, max_len):
    config_for_every_point = []
    for p in points:
        gp = grapple_points[0]
        config = config3Points(spec, gp, p,min_len,max_len)
        if config is not None:
            config_for_every_point.append(config)
    return config_for_every_point


def pointsNearObstruction(spec):
    '''
    Get points near (boundary) of obstruction
    '''
    points_near_obstruction = []
    # Get points on the obstruction
    for m in range(spec.num_obstacles):
        # get the edges of the obstacle:
        for aedge, bedge in spec.obstacles[m].edges:
            # Near the obstruction, sample 3 points
            if aedge[0] == bedge[0]:  # if x is equal
                lx = [aedge[0]] * 5
                ly = list(np.linspace(aedge[1], bedge[1], 7))[1:-1]
            elif aedge[1] == bedge[1]:  # if y is equal
                lx = list(np.linspace(aedge[0], bedge[0], 7))[1:-1]
                ly = [aedge[1]] * 5
            else:
                print("Please check")
            l = []
            for i in range(len(ly)):
                l.append((lx[i], ly[i]))
            points_near_obstruction.extend(l)
    return points_near_obstruction


def pointsBetweenPassage(spec):
    '''
    Get points in
    '''
    # Sample between the passage
    # Get all the edges in a list
    all_edges = []
    for m in range(spec.num_obstacles):
        # get the edges of the obstacle:
        for aedge, bedge in spec.obstacles[m].edges:
            all_edges.append(aedge)
            all_edges.append(bedge)

    all_edges = list(set(all_edges))
    points_between_passage = []
    for e1 in all_edges:
        for e2 in all_edges:
            lx = list(np.linspace(e1[0], e2[0], 4))[1:-1]
            ly = list(np.linspace(e1[1], e2[1], 4))[1:-1]
            l = []
            for i in range(len(ly)):
                l.append((lx[i], ly[i]))
        points_between_passage.extend(l)
    return points_between_passage


def removeUnwantedPoints(points, spec):
    # Remove unwatned points
    # length should not be greater than the sum of all lenght
    # length should not be less than that of min config., that we got by 15 degrees angle
    initial_point = spec.initial.points[0]
    config = spec.initial

    # list the unexpected angle
    unacceptable_angles = [[15, 15, 15], [15, 15, -15], [15, -15, 15], [-15, 15, 15], [15, -15, -15], [-15, 15, -15],
                           [-15, 15, -15], [-15, -15, -15]]

    # for these unexpected angles
    d = [0] * len(unacceptable_angles)
    for an in range(len(unacceptable_angles)):
        config_plus_15 = make_robot_config_from_ee1(config.points[0][0], config.points[0][1], unacceptable_angles[an],
                                                    spec.max_lengths, ee1_grappled=True)
        d[an] = distancePoints(initial_point, config_plus_15.points[-1])

    for p in points:
        # if the distance is greater then maz lenght
        if ((distancePoints(p, initial_point) > sum(spec.max_lengths))
                # or less than min distnace
                or (distancePoints(p, initial_point) < min(d))):
            points.remove(p)
    return points


def randomsample(config_initial, number_samples, spec, graphnodes,a_count):
    """Created sample robot configs and for valid configs creates nodes
    New nodes are checked with existing nodes for neighbours and appended to the neighbours list"""
    configs = []

    # Check if graphnodes is empty(1st iteration) and append source configs to it
    if len(graphnodes) != 0:
        for config in config_initial:
            graphnodes.append(config)
    i = 0
    while i < number_samples:
        # choose a random config from base configs
        random_number = random.randint(0, len(config_initial) - 1)
        config = config_initial[random_number].config

        # choose a random angle and add +value to it
        if config.ee1_grappled == True:
            angles = copy.deepcopy(config.ee1_angles)
        elif config.ee2_grappled == True:
            angles = copy.deepcopy(config.ee2_angles)
        lengths = copy.deepcopy(config.lengths)

        angle_add = random.randint(-50, 50)
        random_number = random.randint(0, len(angles) - 1)
        angles[random_number] = angles[random_number] + angle_add
        random_number = random.randint(0, len(angles) - 1)
        lengths[random_number] = random.uniform(spec.min_lengths[random_number], spec.max_lengths[random_number])

        # Make new config with new angle and length
        if config.ee1_grappled == True:
            config_new = make_robot_config_from_ee1(config.points[0][0], config.points[0][1], angles, config.lengths,
                                                    ee1_grappled=True)
        elif config.ee2_grappled == True:
            config_new = make_robot_config_from_ee2(config.points[len(config.points) - 1][0],
                                                    config.points[len(config.points) - 1][1], angles, config.lengths,
                                                    ee2_grappled=True)

        if (valid_config(config_new, spec) == False):
            continue
        i = i + 1
        configs.append(config_new)

        # Creating a new node with the config and updating other nodes if it is a neighbour

        current_node = copy.deepcopy(GraphNode(spec, config_new))

        for node in graphnodes:
            if (neighbour_check(node, current_node, spec)) != False:
                node.neighbors.append(current_node)
                current_node.neighbors.append(node)

        graphnodes.append(current_node)
    dummy_print=[]
    for i in graphnodes:
        dummy_print.append(i.config)
    write_robot_config_list_to_file("test_output.txt", dummy_print)
    #return graphnodes
    
    sampling_specific_points = pointsBetweenPassage(spec)
    sampling_specific_points.extend(pointsNearObstruction(spec))
    sampling_specific_points = list(set(sampling_specific_points))
    sampling_specific_points = removeUnwantedPoints(sampling_specific_points, spec)
    
    if a_count==3:
        if config.ee1_grappled == True:
            config_for_every_point = config3Obstacle(sampling_specific_points, spec, spec.grapple_points, spec.min_lengths, spec.max_lengths )
        elif config.ee2_grappled == True:            
            grapple_points = copy.deepcopy(spec.grapple_points)
            grapple_points.reverse()
            min_lengths = copy.deepcopy(spec.min_lengths)
            min_lengths.reverse()
            max_lengths = copy.deepcopy(spec.max_lengths)
            max_lengths.reverse()
            config_for_every_point = config3Obstacle(sampling_specific_points, spec, grapple_points, min_lengths, max_lengths )
    elif a_count==4:
        if config.ee1_grappled == True:
            config_for_every_point = config4Obstacle(sampling_specific_points, spec,spec.grapple_points, spec.min_lengths, spec.max_lengths)
        elif config.ee2_grappled == True:            
            grapple_points = copy.deepcopy(spec.grapple_points)
            grapple_points.reverse()
            min_lengths = copy.deepcopy(spec.min_lengths)
            min_lengths.reverse()
            max_lengths = copy.deepcopy(spec.max_lengths)
            max_lengths.reverse()
            config_for_every_point = config4Obstacle(sampling_specific_points, spec, grapple_points, min_lengths, max_lengths )   
    else:
        return graphnodes
    
    for random_config in config_for_every_point:
        if random_config== []:
            continue
        else:
            random_config=random_config[0]
        configs.append(random_config)
        current_node = copy.deepcopy(GraphNode(spec, random_config))
        for node in graphnodes:
            if (neighbour_check(node, current_node, spec)) != False:
                node.neighbors.append(current_node)
                current_node.neighbors.append(node)
        graphnodes.append(current_node)   
    return graphnodes
    

def valid_config(config, spec):
    """
    #Imported from tester
    #Self Collision
    #Bound check
    #Angle constraints
    #Length constraints
    #Obstacle collision
    """
    if test_self_collision(config, spec) == False:
        return False
    if test_environment_bounds(config) == False:
        return False
    if test_angle_constraints(config, spec) == False:
        return False
    if test_length_constraints(config, spec) == False:
        return False
    if test_obstacle_collision(config, spec, spec.obstacles) == False:
        return False
    return True


def neighbour_check(node_1, node_2, spec):
    """Checks if two nodes are neighbours"""

    configs = [node_1.config, node_2.config]
    found_config_list = False

    while found_config_list != True:
        new_config_list = []
        new_config_list.append(configs[0])
        found_config_list = True

        for i in range(len(configs) - 1):
            # Check if the distnace between configs is less than 0.05
            if distance_between_configs_points(configs[i], configs[i + 1]) <= .05:
                new_config_list.append(configs[i + 1])
            else:
                # Find mid point and append to the list of configs between the two configs
                mid_config = mid_point(configs[i], configs[i + 1])
                if valid_config(mid_config, spec) == False:
                    return False
                new_config_list.append(mid_config)
                new_config_list.append(configs[i + 1])
                found_config_list = False
        configs = new_config_list

        # Break loop if the number of configs inbetween exceeds
        if len(configs) >= 100:
            return False
    return configs


def distance_between_configs_points(config_1, config_2):
    """Returns maximum distance between config points"""
    sum = 0
    for i in range(len(config_1.points)):
        if sum < distancePoints(config_1.points[i], config_2.points[i]):
            sum = distancePoints(config_1.points[i], config_2.points[i])
    return sum


def distancePoints(a, b):
    """Returns the distance between two points"""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def mid_point(config_1, config_2):
    """Return the mid config for """
    angles_new = []
    length_new = []
    if config_1.ee1_grappled == True:
        angles_1 = copy.deepcopy(config_1.ee1_angles)
    elif config_1.ee2_grappled == True:
        angles_1 = copy.deepcopy(config_1.ee2_angles)

    if config_2.ee1_grappled == True:
        angles_2 = copy.deepcopy(config_2.ee1_angles)
    elif config_2.ee2_grappled == True:
        angles_2 = copy.deepcopy(config_2.ee2_angles)

    for i in range(len(angles_1)):
        an = (angles_1[i]) / 2 + (angles_2[i]) / 2
        angles_new.append(an)

    for i in range(len(config_1.lengths)):
        length_new.append((config_1.lengths[i] + config_2.lengths[i]) / 2)

    if config_1.ee1_grappled == True:
        config = make_robot_config_from_ee1(config_1.points[0][0], config_1.points[0][1], angles_new, length_new,
                                        ee1_grappled=True)
    elif config_1.ee2_grappled == True:
        config = make_robot_config_from_ee2(config_1.points[len(config_1.points) - 1][0], config_1.points[len(config_1.points) - 1][1], angles_new, length_new,
                                            ee2_grappled=True)

    return config


def neighbour_check_configs(node_1, node_2, spec):
    configs = [node_1, node_2]
    found_config_list = False
    while found_config_list != True:
        # print("found path")
        new_config_list = []
        new_config_list.append(configs[0])
        found_config_list = True
        for i in range(len(configs) - 1):
            # check if the if path between n1 and n2 is collision free:
            if test_config_distance(configs[i], configs[i + 1], spec):
                new_config_list.append(configs[i + 1])
                # return configs
            else:
                mid_config = mid_point(configs[i], configs[i + 1])
                if valid_config(mid_config, spec) == False:
                    return False
                new_config_list.append(mid_config)
                new_config_list.append(configs[i + 1])
                found_config_list = False
        configs = new_config_list
    return configs


def path_between_points(initial, goal, spec, number_of_iterations,a_count):
    init_node = GraphNode(spec, initial)
    goal_node = GraphNode(spec, goal)

    GraphNodes = []  # List of all nodes created

    found_result = False
    for i in range(number_of_iterations):
        print("iteration number: ", i)

        # Create Random Samples
        random_graph_nodes = randomsample([init_node, goal_node], 100, spec, GraphNodes,a_count)

        # Building state graph
        init_container = [init_node]
        init_visited = {init_node: [init_node.config]}
        while len(init_container) > 0:
            current = init_container.pop(0)
            if test_config_equality(current.config, goal, spec):
                # found path to goal
                print("Finally :)")
                final_config = init_visited[current]

                # Build small neighbours
                print_configs = []
                for i in range(len(final_config) - 1):
                    neigh = neighbour_check_configs(final_config[i], final_config[i + 1], spec)
                    if neigh:
                        for j in neigh:
                            print_configs.append(j)
                    else:
                        print("Something is just not right")
                print_configs.append(final_config[len(final_config) - 1])
                return print_configs

            # Check each successor to check if previously visited node
            successors = current.get_successors()
            for suc in successors:
                if suc not in init_visited:
                    init_container.append(suc)
                    init_visited[suc] = init_visited[current] + [suc.config]
    return None


def main(arguments):
    start_time = time.time()
    # Creating problemspec and (initial,goal nodes) from given file
    space = ProblemSpec(arguments[0])
    spec = space
    initial_config = space.initial
    goal_config = space.goal
    g_count=len(space.grapple_points)
    a_count=len(space.min_lengths)

    if len(spec.grapple_points) == 1:
        path_final = path_between_points(initial_config, goal_config, spec, 10,a_count)

    grapple_points=spec.grapple_points
    if len(spec.grapple_points)==2:
        random_configs=[]
        if a_count==3:
            while random_configs==[]:
                random_configs=config3Points(spec,grapple_points[0],grapple_points[1],  spec.min_lengths, spec.max_lengths)
        elif a_count==4:
            while random_configs == []:
                random_configs =config4Points(spec, grapple_points[0], grapple_points[1],spec.min_lengths, spec.max_lengths)
        else:
            print("Configured for 3 and 4 arms only")
        for config in random_configs:
            path_1 = path_between_points(initial_config, config, spec,10,a_count)
            config.ee2_grappled=True
            config.ee1_grappled = False
            path_2 = path_between_points( config,goal_config, spec,20,a_count)
            if path_1 != None and path_2!= None:
                path_final=path_1+path_2
                break

    if len(grapple_points) == 3:
        bridge_1=[]
        if a_count == 3:
            while bridge_1 == []:
                bridge_1 = config3Points(spec, grapple_points[0], grapple_points[1],spec.min_lengths, spec.max_lengths)
                if bridge_1!=[]:
                    bridge_1 = bridge_1[0]
        elif a_count == 4:
            while bridge_1 == []:
                bridge_1 = config4Points(spec, grapple_points[0], grapple_points[1],spec.min_lengths, spec.max_lengths)
        print(bridge_1)
        path_1 = path_between_points(initial_config, bridge_1, spec, 10,a_count)
        print("found path_1")
        write_robot_config_list_to_file("test_output.txt", path_1)
        copy_bridge_1 = copy.deepcopy(bridge_1)
        copy_bridge_1.ee1_grappled = False
        copy_bridge_1.ee2_grappled = True
        bridge_2 = []
        if a_count == 3:
            while bridge_2 == []:
                bridge_2 = config3Points(spec, grapple_points[2], grapple_points[1],spec.min_lengths, spec.max_lengths)
                if bridge_2 != []:
                    bridge_2=bridge_2[0]
        elif a_count == 4:
            while bridge_2 == []:
                bridge_2 = config4Points(spec, grapple_points[2], grapple_points[1],spec.min_lengths, spec.max_lengths)
        bridge_2.ee1_grappled = False
        bridge_2.ee2_grappled = True
        print("bridge 1", copy_bridge_1)
        print("bridge 2", bridge_2)
        path_2 = path_between_points(copy_bridge_1, bridge_2, spec, 10,a_count)
        print("found path_2")
        copy_bridge_2 = copy.deepcopy(bridge_2)
        copy_bridge_2.ee1_grappled = True
        copy_bridge_2.ee2_grappled = False
        path_3 = path_between_points(copy_bridge_2, goal_config, spec, 10,a_count)
        print("found path_3")
        path_final = path_1 + path_2 + path_3
    #dummy = randomsample([GraphNode(spec, dummy_config)], 100, spec, [])
    write_robot_config_list_to_file(arguments[1], path_final)
    end_time = time.time()
    time_rec = end_time - start_time
    print(time_rec)
    
    
if __name__ == '__main__':
    main(sys.argv[1:]) 
