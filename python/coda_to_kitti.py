"""
This script converts the poses in coda (ts x y z qw qx qy qz) to kitti format 
(r1 r2 r3 t1 r4 r5 r6 t2 r7 r8 r9 t3) where r1 r2 r3 are the first row of the rotation matrix,

"""

import os
import numpy as np
from scipy.spatial.transform import Rotation as R

import argparse

def parse_args():
    parser = argparse.ArgumentParser(description='Converts coda poses to kitti format')
    parser.add_argument('--indir', type=str, default='/media/arthur/ExtremePro/CODa_v2', help='Directory containing the coda poses')
    parser.add_argument('--outdir', type=str, default='/media/arthur/ExtremePro/removert/coda', help='Directory to save the kitti poses')
    parser.add_argument('--seq', type=str, default='0', help='Sequence number')
    return parser.parse_args()

def pose_coda2kitti(in_path, out_path):
    """
    Converts coda poses to kitti format and saves them in specific directory
    """
    poses = np.loadtxt(in_path, dtype=np.float64)
    kitti_poses = []
    for i in range(poses.shape[0]):
        pose = poses[i]
        kitti_pose = np.zeros((3, 4))
        kitti_pose[:3, :3] = R.from_quat([pose[5], pose[6], pose[7], pose[4]]).as_matrix() # qw qx qy qz
        kitti_pose[:3, 3] = pose[1:4]

        kitti_poses.append(kitti_pose.flatten())
    kitti_poses = np.array(kitti_poses)
    np.savetxt(out_path, kitti_poses)
    print(f'Saved poses in {out_path}')

def convert_coda2kitti(indir, outdir, seq):
    """
    Converts poses to kitti format and saves them in specific directory
    """
    print(f'---- Converting coda poses to kitti format for sequence {seq} ----')
    in_path = os.path.join(indir, 'poses', 'dense_global', f'{seq}.txt')
    out_path = os.path.join(outdir, 'poses', f'{seq}.txt')
    if not os.path.exists(os.path.dirname(out_path)):
        out_pathdir = os.path.dirname(out_path)
        print(f'Creating directory {out_pathdir}')
        os.makedirs(out_pathdir)
    
    print(f'---- Loading poses from {in_path} and saving to {out_path} ----')
    pose_coda2kitti(in_path, out_path)

if __name__ == "__main__":
    args = parse_args()
    assert args.seq not in ['8', '14', '15'], f'Global poses for {args.seq} are not available in CODa'
    convert_coda2kitti(args.indir, args.outdir, args.seq)
