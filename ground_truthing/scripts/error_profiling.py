#!/usr/bin/env python

import rosbag

import csv
import plotly.graph_objs as go
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.stats import norm

def ProcessBag(path_to_bag):

    bag = rosbag.Bag(path_to_bag)

    target_poses = []
    chaser_poses = []
    optical_poses = []
    usbl_positions = []

    for topic, msg, t in bag.read_messages(topics=['/service_boat/dr/odom',
                                                   '/service_boat/enu/usbl_fix',
                                                   '/lolo/dr/odom',
                                                   '/lolo/perception/optical_pose']):
        if topic == '/service_boat/dr/odom':
            target_poses.append({'time': t.to_sec(), 'pose' : PoseToMatrix(msg.pose)})
        elif topic == '/service_boat/enu/usbl_fix':
            usbl_positions.append({'time': t.to_sec(), 'pose': PosToVector(msg.pose.position)})
        elif topic == '/lolo/dr/odom':
            chaser_poses.append({'time': t.to_sec(), 'pose' : PoseToMatrix(msg.pose)})
        elif topic == '/lolo/perception/optical_pose':
            optical_poses.append({'time': t.to_sec(), 'pose' : PoseToMatrix(msg.pose)})

    return target_poses, chaser_poses, optical_poses, usbl_positions

def pose_to_timestamped_dict(t, msg):
    return {
        'time': t.to_sec(),
        'x': msg.pose.position.x,
        'y': msg.pose.position.y,
        'z': msg.pose.position.z,
        'qx': msg.pose.orientation.x,
        'qy': msg.pose.orientation.y,
        'qz': msg.pose.orientation.z,
        'qw': msg.pose.orientation.w,
    }

def position_to_timestamped_dict(t, msg):
    return {
        'time': t.to_sec(),
        'x': msg.x,
        'y': msg.y,
        'z': msg.z,
    }

def extract_time(data):
  return [e['time'] for e in data]

def find_closest_time_index(t, times):
  return np.argmin(np.abs(t - np.array(times)))

def build_timestamped_pairs(reference_poses, poses):
  timestamped_pairs = []

  times = extract_time(poses)
  ref_times = extract_time(reference_poses)
  closest_t_idx = 0
  for i,t in enumerate(times):
    print("in {0} of {1}".format(i, len(times)))
    closest_t_idx = find_closest_time_index(t, ref_times[closest_t_idx:])
    ref_state = reference_poses[closest_t_idx]
    state = poses[i]
    timestamped_pairs.append( (t, ref_state, state) )

  return timestamped_pairs

def PoseToMatrix(pose):
    """
    Pose msg to 4x4 numpy array.
    """
    rot_matrix = Rotation.from_quat([pose.pose.orientation.x,
                                     pose.pose.orientation.y,
                                     pose.pose.orientation.z,
                                     pose.pose.orientation.w])
    translation = np.array([[pose.pose.position.x],[pose.pose.position.y],
                            [pose.pose.position.z]])

    transformation = np.eye(4)
    transformation[0:3,0:3] = rot_matrix.as_matrix()
    transformation[0:-1,-1:] = translation

    return transformation

def PosToVector(pos):
    """
    Pose msg to 3X1 numpy array.
    """
    position = np.array([[pos.x],[pos.y],
                        [pos.z]])

    return position

def GetRelativePose(pose_i, pose_j):
    """
    Return the relative pose delta_ij = pose_j (-) pose_i,
    where pose_j,i are homogeneous transformation in a 4x4
    numpy array.
    """
    inv_rot_i = pose_i[0:-1,0:-1].transpose()
    inv_trans_i = - (inv_rot_i @ pose_i[0:-1,-1:])

    inv_pose_i = np.eye(4)
    inv_pose_i[0:-1,0:-1] = inv_rot_i
    inv_pose_i[0:-1,-1:] = inv_trans_i

    return inv_pose_i @ pose_j

def GetPoseErrors(pose, true_pose):
    translation_error = pose[:-1,-1:] - true_pose[:-1,-1:]
    rotation_error = Rotation.from_matrix(pose[:-1,:-1]
                                          @ true_pose[:-1,:-1].transpose())

    return (translation_error, rotation_error.as_euler('xyz'))

def GetOpticalPoseErrors(triplets):
    errors = []

    # Camera extrinsics, from lolo_description.
    trans = np.array([[-0.6561], [0.003], [0.4935]])
    rot = Rotation.from_euler("xyz", [-1.5707, 0, 1.5707])
    xc_tfm_cam = np.eye(4)
    xc_tfm_cam[0:-1,0:-1] = rot.as_matrix()
    xc_tfm_cam[0:-1,-1:] = trans

    # Light fiducials extrinsics (sbg->lights), from boat_description.
    trans = np.array([[0.49], [0.0], [0.4935]])
    rot = Rotation.from_euler("xyz", [1.5707, 0, -1.5707])
    inv_rot = rot.as_matrix().transpose()
    inv_trans = - (inv_rot @ trans)
    lf_tfm_sbg = np.eye(4)
    lf_tfm_sbg[0:-1,0:-1] = inv_rot
    lf_tfm_sbg[0:-1,-1:] = inv_trans

    # SBG->base_link, from service_boat_odometer.launch.
    rot = Rotation.from_euler("xyz", [3.14159, 0, 1.5707])
    sbg_tfm_xt = np.eye(4)
    sbg_tfm_xt[0:-1,0:-1] = rot.as_matrix().transpose()

    meas = []
    relatives = []
    for p in triplets:
        xc = p[1][1]["pose"]
        xt = p[1][2]["pose"]
        # Compose with camera extrinsics and light fiducials extrinsics.
        # z = xc_tfm_cam @ p[2]["pose"]
        z = xc_tfm_cam @ p[2]["pose"] @ lf_tfm_sbg @ sbg_tfm_xt
        meas.append(z)
        xc_tfm_xt = GetRelativePose(xc, xt)
        relatives.append(xc_tfm_xt)
        errors.append(GetRelativePose(xc_tfm_xt, z))

    return errors, meas, relatives

def plot_3d_rel_cams(meas, rels):
    fig = go.Figure()

    if len(meas) != len(rels):
        print("La cagaste de tamanios wey!")
        return

    cams_x = []
    cams_y = []
    cams_z = []

    odom_x = []
    odom_y = []
    odom_z = []

    for i in range(len(meas)):
        c = meas[i]
        o = rels[i]

        rot_cam = c[0:3,0:3].transpose()
        trans_cam = c[0:3,3:]
        cam_p = rot_cam @ trans_cam

        cams_x.append(cam_p[0,0])
        cams_y.append(cam_p[1,0])
        cams_z.append(cam_p[2,0])

        rot_odom = o[0:3,0:3].transpose()
        trans_odom = o[0:3,3:]
        odom_p = rot_odom @ trans_odom

        odom_x.append(odom_p[0,0])
        odom_y.append(odom_p[1,0])
        odom_z.append(odom_p[2,0])

    fig.add_trace(go.Scatter3d(x=cams_x,
                               y=cams_y,
                               z=cams_z,
                               mode='markers',
                               name='cam'))
    fig.add_trace(go.Scatter3d(x=odom_x,
                               y=odom_y,
                               z=odom_z,
                               mode='markers',
                               name='odom'))

    return fig



def GetUsblPosErrors(triplets):
    errors = []

    for p in triplets:
        xc = p[1][1]["pose"]
        xt = p[1][2]["pose"]
        z = p[2]["pose"]
        xt_tfm_xc = GetRelativePose(xt, xc)
        errors.append(xt_tfm_xc[0:-1, -1:] - z)

    return errors

def GetUsblGlobalErrors(triplets):
    errors = []

    meas = []
    rels = []
    for p in triplets:
        xc = p[1][1]["pose"]
        xt = p[1][2]["pose"]
        z_hat = p[2]["pose"]
        z = z_hat - xt[:-1, -1:]
        meas.append(z)
        xt_tfm_xc = GetRelativePose(xt, xc)
        rels.append(xt_tfm_xc[0:-1,-1:])
        errors.append(xt_tfm_xc[0:-1, -1:] - z)

    return errors, meas, rels

def SyncMeasurementsAndPoses(agent_pairs, meas):
    synced_triplets = []

    times = extract_time(meas)
    ref_times = [p[0] for p in agent_pairs]
    for i,t in enumerate(times):
        closest_t_idx = find_closest_time_index(t, ref_times)
        agent_states = agent_pairs[closest_t_idx]
        measurement = meas[i]
        synced_triplets.append((t, agent_states, measurement))

    return synced_triplets

def CartToSphere(p):

    bearing = np.arctan2(p[1,0], p[0,0])
    elevation = np.arctan2(p[2,0], np.sqrt(p[1,0]**2 + p[0,0]**2))
    distance = np.linalg.norm(p)
    return np.array([distance,
                     np.rad2deg(bearing),
                     np.rad2deg(elevation)])


def GetUsblSphericalErrors(triplets):
    errors = []

    meas = []
    rels = []
    for p in triplets:
        xc = p[1][1]["pose"]
        xt = p[1][2]["pose"]
        z_hat = p[2]["pose"]
        z = z_hat - xt[:-1, -1:]
        meas.append(z)
        xt_tfm_xc = GetRelativePose(xt, xc)
        rels.append(CartToSphere(xt_tfm_xc[0:-1, -1:]))
        errors.append(CartToSphere(xt_tfm_xc[0:-1, -1:]) - CartToSphere(z))

    return errors, meas, rels

def GetAgentPosePairs(target_poses, chaser_poses):
    return build_timestamped_pairs(chaser_poses, target_poses)

def SaveCsvs(optical_errors, optical_measured, optical_relative,
             usbl_errors, usbl_measured, usbl_relative,
             usbl_sphere_errors, usbl_sphere_meas, usbl_sphere_rels):

    filename = "usbl_errors.csv"
    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)

        # Write the header row
        writer.writerow(["x_rel", "y_rel", "z_rel",
                         "r_rel", "b_rel", "e_rel",
                         "x_meas", "y_meas", "z_meas",
                         "r_meas", "b_meas", "e_meas",
                         "x_err", "y_err", "z_err",
                         "r_err", "b_meas", "e_meas"])

        # Write the data rows.
        for i in range(len(usbl_errors)):
            try:
                writer.writerow([usbl_relative[i][0,0], usbl_relative[i][1,0], usbl_relative[i][2,0],
                                usbl_sphere_rels[i][0], usbl_sphere_rels[i][1], usbl_sphere_rels[i][2],
                                usbl_measured[i][0,0], usbl_measured[i][1,0], usbl_measured[i][2,0],
                                usbl_sphere_meas[i][0,0], usbl_sphere_meas[i][1,0], usbl_sphere_meas[i][2,0],
                                usbl_errors[i][0,0], usbl_errors[i][1,0], usbl_errors[i][2,0],
                                usbl_sphere_errors[i][0], usbl_sphere_errors[i][1], usbl_sphere_errors[i][2]])
            except:
                print("usbl index error?")
                continue

    filename = "optical_errors.csv"
    with open(filename, "w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow(["qx_rel", "qy_rel", "qz_rel", "qw_rel",
                         "x_rel", "y_rel", "z_rel",
                         "qx_meas", "qy_meas", "qz_meas", "qw_meas",
                         "x_meas", "y_meas", "z_meas",
                         "qx_err", "qy_err", "qz_err", "qw_err",
                         "x_err", "y_err", "z_err"])

        opt_rels_rots = [Rotation.from_matrix(e[0:-1,0:-1]) for e in optical_relative]
        opt_meas_rots = [Rotation.from_matrix(e[0:-1,0:-1]) for e in optical_measured]
        opt_error_rots = [Rotation.from_matrix(e[0:-1,0:-1]) for e in optical_errors]

        for i in range(len(optical_errors)):
            try:
                q_rel = opt_rels_rots[i].as_quat()
                q_meas = opt_meas_rots[i].as_quat()
                q_err = opt_error_rots[i].as_quat()

                writer.writerow([q_rel[0], q_rel[1], q_rel[2], q_rel[3],
                                optical_relative[i][0,3], optical_relative[i][1,3], optical_relative[i][2,3],
                                q_meas[0], q_meas[1], q_meas[2], q_meas[3],
                                optical_measured[i][0,3], optical_measured[i][1,3], optical_measured[i][2,3],
                                q_err[0], q_err[1], q_err[2], q_err[3],
                                optical_errors[i][0,3], optical_errors[i][1,3], optical_errors[i][2,3]])
            except:
                print("Optical index error?")
                continue

def GetAllErrors(target_poses, chaser_poses, optical_poses, usbl_positions):
    # I'm gonna assume there will always be more chaser poses.
    if len(target_poses) > len(chaser_poses):
        print("Something is wrong!")

    # Let's build chaser-target pose pairs first and then look for the closer one
    # wrt the camera and the USBL times.
    agent_pose_pairs = build_timestamped_pairs(chaser_poses, target_poses)

    # Now optical + chaser + target throuples.
    optical_triplets = SyncMeasurementsAndPoses(agent_pose_pairs, optical_poses)

    # Now usbl + chaser + target throuples.
    usbl_triplets = SyncMeasurementsAndPoses(agent_pose_pairs, usbl_positions)

    # Calculate the errors.
    optical_errors, optical_measured, optical_relative = GetOpticalPoseErrors(optical_triplets)
    usbl_errors, usbl_measured, usbl_relative = GetUsblGlobalErrors(usbl_triplets)
    usbl_sphere_errors, usbl_sphere_meas, usbl_sphere_rels = GetUsblSphericalErrors(usbl_triplets)

    return (optical_errors, optical_measured, optical_relative,
             usbl_errors, usbl_measured, usbl_relative,
             usbl_sphere_errors, usbl_sphere_meas, usbl_sphere_rels)

    # SaveCsvs(optical_errors, optical_measured, optical_relative,
             # usbl_errors, usbl_measured, usbl_relative,
             # usbl_sphere_errors, usbl_sphere_meas, usbl_sphere_rels)

    # return (optical_errors, usbl_errors, usbl_sphere_errors)

def PlotUsblErrorHistograms(usbl_errors, usbl_sphere_errors):

    # Plot usbl errors.
    bins = 50
    trid_usbl_errors = [np.sum(e) for e in usbl_errors]
    plane_usbl_errors = [np.sum(e[:-1]) for e in usbl_errors]
    z_usbl_errors = [e[-1,0] for e in usbl_errors]

    fig, ax= plt.subplots(1,4)

    fig.suptitle("USBL position errors (cartesian)")

    ax[0].hist(trid_usbl_errors, bins, facecolor='g')
    ax[0].set_title("3D error")
    ax[0].set_xlabel("error [m]")
    ax[0].grid()

    ax[1].hist(plane_usbl_errors, bins, facecolor='b')
    ax[1].set_title("2D plane errors")
    ax[1].set_xlabel("error [m]")
    ax[1].grid()

    ax[2].hist(z_usbl_errors, bins, facecolor='r')
    ax[2].set_title("Z errors")
    ax[2].set_xlabel("error [m]")
    ax[2].grid()

    ax[3].hist(z_usbl_errors, bins, label="z", facecolor='r', alpha=0.7)
    ax[3].hist(plane_usbl_errors, bins, label="2d", facecolor='b', alpha=0.7)
    ax[3].hist(trid_usbl_errors, bins, label="3d", facecolor='g', alpha=0.7)
    ax[3].set_title("all errors")
    ax[3].set_xlabel("error [m]")
    ax[3].grid()
    ax[3].legend()

    x_min = min(ax[0].get_xlim()[0], ax[1].get_xlim()[0],
                ax[2].get_xlim()[0], ax[3].get_xlim()[0])
    x_max = max(ax[0].get_xlim()[1], ax[1].get_xlim()[1],
                ax[2].get_xlim()[1], ax[3].get_xlim()[1])
    y_min = min(ax[0].get_ylim()[0], ax[1].get_ylim()[0],
                ax[2].get_ylim()[0], ax[3].get_ylim()[0])
    y_max = max(ax[0].get_ylim()[1], ax[1].get_ylim()[1],
                ax[2].get_ylim()[1], ax[3].get_ylim()[1])
    for a in ax:
        a.set_xlim(x_min, x_max)
        a.set_ylim(y_min, y_max)

    # Let's calculate the spherical errors as well.
    # usbl_sphere_errors = CartToSphere(usbl_errors)
    range_errors = [e[0] for e in usbl_sphere_errors]
    r_mu, r_sigma = norm.fit(range_errors)

    bearing_errors = [e[1] for e in usbl_sphere_errors]
    b_mu, b_sigma = norm.fit(bearing_errors)

    elevation_errors = [e[2] for e in usbl_sphere_errors]
    e_mu, e_sigma = norm.fit(elevation_errors)

    fig2, ax2 = plt.subplots(1,3)
    fig2.suptitle("USBL position errors (spherical)")

    ax2[0].hist(range_errors, bins, facecolor='g')
    ax2[0].set_title("range error")
    ax2[0].set_xlabel("error [m]")
    ax2[0].grid()

    ax2[1].hist(bearing_errors, bins, facecolor='b')
    ax2[1].set_title("bearing error")
    ax2[1].set_xlabel("error [deg]")
    ax2[1].grid()

    ax2[2].hist(elevation_errors, bins, facecolor='r')
    ax2[2].set_title("elevation error")
    ax2[2].set_xlabel("error [deg]")
    ax2[2].grid()

    plt.show()

def PlotOpticalErrorHistograms(optical_errors):

    bins = 180
    position_error = [np.sum(e[:-1,-1:]) for e in optical_errors]
    rot_error_scipy = [Rotation.from_matrix(e[0:-1,0:-1]) for e in optical_errors]
    rot_error_euler = [e.as_euler("xyz") for e in rot_error_scipy]
    roll_error = [np.rad2deg(e[0]) for e in rot_error_euler]
    pitch_error = [np.rad2deg(e[1]) for e in rot_error_euler]
    yaw_error = [np.rad2deg(e[2]) for e in rot_error_euler]

    fig, ax = plt.subplots(1,4)
    fig.suptitle("optical measurement error")

    ax[0].hist(position_error, bins, facecolor='b')
    ax[0].set_title("position error")
    ax[0].set_xlabel("error [m]")
    ax[0].grid()

    ax[1].hist(roll_error, bins, facecolor='b')
    ax[1].set_title("roll error")
    ax[1].set_xlabel("error [m]")
    ax[1].grid()

    ax[2].hist(pitch_error, bins, facecolor='b')
    ax[2].set_title("pitch error")
    ax[2].set_xlabel("error [m]")
    ax[2].grid()

    ax[3].hist(yaw_error, bins, facecolor='b')
    ax[3].set_title("yaw error")
    ax[3].set_xlabel("error [m]")
    ax[3].grid()

    plt.show()

def PlotAllRosbags():
    bags = ["/media/aldoteran/T5 EVO/datasets/2024-06-11-asko/reprocessed_datasets/2024-06-11-17-28-05_dataset.bag",
            "/media/aldoteran/T5 EVO/datasets/2024-06-11-asko/reprocessed_datasets/2024-06-11-18-10-00_dataset.bag",
            "/media/aldoteran/T5 EVO/datasets/2024-06-11-asko/reprocessed_datasets/2024-06-11-19-05-45_dataset.bag"]

    optical_errors = []
    optical_measured = []
    optical_relative = []
    usbl_errors = []
    usbl_measured = []
    usbl_relative = []
    usbl_sphere_errors = []
    usbl_sphere_measured = []
    usbl_sphere_relative = []
    for bag in bags:
        target_poses, chaser_poses, optical_poses, usbl_positions = ProcessBag(bag)
        opt_err, opt_meas, opt_rel, usbl_err, usbl_meas, usbl_rel, \
        usbl_sphere_err, usbl_sphere_meas, usbl_sphere_rels = GetAllErrors(target_poses, chaser_poses,
                                                                                   optical_poses, usbl_positions)
        optical_errors.extend(opt_err)
        optical_measured.extend(opt_meas)
        optical_relative.extend(opt_rel)
        usbl_errors.extend(usbl_err)
        usbl_measured.extend(usbl_meas)
        usbl_relative.extend(usbl_rel)
        usbl_sphere_errors.extend(usbl_sphere_err)
        usbl_sphere_measured.extend(usbl_sphere_meas)
        usbl_sphere_relative.extend(usbl_sphere_rels)

    return optical_errors, optical_measured, optical_relative, usbl_errors,\
            usbl_measured, usbl_relative, usbl_sphere_errors, usbl_sphere_measured,\
            usbl_sphere_relative

    SaveCsvs(optical_errors, optical_measured, optical_relative,
             usbl_errors, usbl_measured, usbl_relative,
             usbl_sphere_errors, usbl_sphere_measured, usbl_sphere_relative)

    PlotUsblErrorHistograms(usbl_errors, usbl_sphere_errors)
    PlotOpticalErrorHistograms(optical_errors)

# if __name__ == "__main__":
