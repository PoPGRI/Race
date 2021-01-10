import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import pickle

def compute_straight_segment_waypoint(seg_length):
    seg_waypoints = []
    if seg_length == 200:
        res = np.arange(-100,101,5)
        for i in range(len(res)):
            seg_waypoints.append([res[i],-10])
        pass
    elif seg_length == 100:
        res = np.arange(-50,51,5)
        for i in range(len(res)):
            seg_waypoints.append([res[i],-10])
        pass
    elif seg_length == 50:
        res = np.arange(-25,26,5)
        for i in range(len(res)):
            seg_waypoints.append([res[i],-10])
        pass
    return seg_waypoints

def compute_curve_segment_waypoint(seg_length):
    seg_waypoints = []
    if seg_length == 100:
        res = np.arange(-np.pi/2,0.01,np.pi/80)
        center_x = -52.7
        center_y = 52.7
        r = 52.7+47.2    
        for i in range(len(res)):
            x = center_x + r*np.cos(res[i])
            y = center_y + r*np.sin(res[i])
            seg_waypoints.append([x,y])
        pass
    elif seg_length == 50:
        res = np.arange(-np.pi/2,0.01,np.pi/40)
        center_x = -27.7
        center_y = 27.7
        r = 27.7+22.2    
        for i in range(len(res)):
            x = center_x + r*np.cos(res[i])
            y = center_y + r*np.sin(res[i])
            seg_waypoints.append([x,y])
        pass
    return seg_waypoints

def transform_waypoints(orig_waypoints, transform_vector):
    tmp = np.array(orig_waypoints)
    trans_x = transform_vector[0]
    trans_y = transform_vector[1]
    trans_theta = transform_vector[5]
    new_x = np.cos(trans_theta)*tmp[:,0] - np.sin(trans_theta)*tmp[:,1]
    new_y = np.sin(trans_theta)*tmp[:,0] + np.cos(trans_theta)*tmp[:,1]
    new_x = np.expand_dims(new_x + trans_x, axis = 1)
    new_y = np.expand_dims(new_y + trans_y, axis = 1)

    transformed_waypoints = np.concatenate((new_x, new_y),axis = 1).tolist()
    return transformed_waypoints

def generate_waypoint(fn, start_x, start_y):
    waypoints = []
    un_ordered_waypoints = []
    start_end = []
    tree = ET.parse(fn)
    root = tree.getroot()
    states = root[0].findall('state')[0]
    models = states.findall('model')
    for model in models:
        name = model.attrib['name']
        name = name.split(' ')
        if name[0] == '(DS)':
            seg_length = int(name[-1].split('m')[0])
            if name[1] == 'Curved':
                seg_waypoints = compute_curve_segment_waypoint(seg_length)
            elif name[1] == 'Straight':
                seg_waypoints = compute_straight_segment_waypoint(seg_length)
            tmp = model.findall('pose')[0].text
            transform_vector = [float(i) for i in tmp.split(' ')]
            transformed_waypoints = transform_waypoints(seg_waypoints, transform_vector)
            # waypoints = waypoints + transformed_waypoints
    
            un_ordered_waypoints.append(transformed_waypoints)
            start_end.append(transformed_waypoints[0])
            start_end.append(transformed_waypoints[-1])
    
    seg = un_ordered_waypoints[0]
    start_end = np.array(start_end)
    for i in range(len(un_ordered_waypoints)-1):
        waypoints += seg[:-1]
        end = seg[-1]
        res = (start_end[:,0]-end[0])**2 + (start_end[:,1]-end[1])**2
        idx = np.argpartition(res,2)[:2]
        idx = idx[np.argmax(res[idx])]
        if idx%2 == 1:
            idx = int(idx/2)
            seg = un_ordered_waypoints[idx]
            seg.reverse()
        else:
            idx = int(idx/2)
            seg = un_ordered_waypoints[idx]

    min_dist = 1e10
    min_idx = -1
    for i in range(len(waypoints)):
        dist = (waypoints[i][0]-start_x)**2 + (waypoints[i][1]-start_y)**2
        if dist < min_dist:
            min_dist = dist
            min_idx = i

    waypoints = waypoints[min_idx:] + waypoints[:min_idx]

    return waypoints[1:]

if __name__ == "__main__":
    fn = "gem_simulator/gem_gazebo/worlds/mp5_4.world"
    waypoints = np.array(generate_waypoint(fn,0,-98))
    plt.plot(waypoints[:,0],waypoints[:,1])
    plt.plot(waypoints[:,0],waypoints[:,1],'.')
    plt.plot(0,-98,'.')
    

    plt.show()

    pickle.dump(waypoints, open("waypoints", "wb"))