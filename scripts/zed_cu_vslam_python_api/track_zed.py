import numpy as np
from pathlib import Path
from PIL import Image
import cuvslam
import rerun as rr

# DATASET_DIR = Path("/home/delmario/ORB_SLAM3/runs/2026-04-02_12-27-14")
# DATASET_DIR = Path("/home/delmario/ORB_SLAM3/runs/2026-04-02_09-54-24")
DATASET_DIR = Path("/home/delmario/ORB_SLAM3/runs/2026-04-14_14-09-05")
OUTPUT_TRAJ = DATASET_DIR / "cuvslam_zed_traj.txt"

FX = 520.890747
FY = 520.890747
CX = 639.319397
CY = 355.391296
WIDTH = 1280
HEIGHT = 720
BASELINE = 0.120019
RECTIFIED = True


def load_image(path: Path):
    img = Image.open(path)
    arr = np.array(img)
    return np.ascontiguousarray(arr)


def build_rig():
    left = cuvslam.Camera()
    left.focal = [FX, FY]
    left.principal = [CX, CY]
    left.size = [WIDTH, HEIGHT]
    left.distortion = cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole, [])
    left.rig_from_camera = cuvslam.Pose(
        rotation=[0, 0, 0, 1],
        translation=[0, 0, 0]
    )

    right = cuvslam.Camera()
    right.focal = [FX, FY]
    right.principal = [CX, CY]
    right.size = [WIDTH, HEIGHT]
    right.distortion = cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole, [])
    right.rig_from_camera = cuvslam.Pose(
        rotation=[0, 0, 0, 1],
        translation=[+BASELINE, 0, 0]
    )

    rig = cuvslam.Rig()
    rig.cameras = [left, right]
    rig.imus = []
    return rig


def load_pairs(dataset_dir: Path):
    ts_file = dataset_dir / "timestamps.txt"
    left_dir = dataset_dir / "left"
    right_dir = dataset_dir / "right"

    pairs = []
    with open(ts_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            ts_str, fname = line.split()
            ts_ns = int(float(ts_str) * 1e9)

            left_img = left_dir / fname
            right_img = right_dir / fname

            if left_img.exists() and right_img.exists():
                pairs.append((ts_ns, left_img, right_img))
            else:
                print(f"Par ausente: {fname}")

    return pairs


def main():
    rig = build_rig()

    cfg = cuvslam.Tracker.OdometryConfig(
        async_sba=False,
        rectified_stereo_camera=RECTIFIED,
        odometry_mode=cuvslam.Tracker.OdometryMode.Multicamera
    )

    tracker = cuvslam.Tracker(rig, cfg)

    pairs = load_pairs(DATASET_DIR)
    print(f"{len(pairs)} pares encontrados")

    rr.init("cuVSLAM ZED Folder", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)

    traj = []
    traj_rows = []

    for i, (ts_ns, left_path, right_path) in enumerate(pairs):
        left = load_image(left_path)
        right = load_image(right_path)

        odom_pose_estimate, _ = tracker.track(ts_ns, [left, right])

        if odom_pose_estimate.world_from_rig is None:
            print(f"Frame {i}: tracking falhou", flush=True)
            continue

        pose = odom_pose_estimate.world_from_rig.pose
        tx, ty, tz = map(float, pose.translation)
        qx, qy, qz, qw = map(float, pose.rotation)

        traj.append([tx, ty, tz])
        traj_rows.append([ts_ns * 1e-9, tx, ty, tz, qx, qy, qz, qw])

        rr.set_time_sequence("frame", i)

        observations_left = tracker.get_last_observations(0)
        observations_right = tracker.get_last_observations(1)

        if observations_left:
            points_left = np.array([[obs.u, obs.v] for obs in observations_left])
            colors_left = np.array([
                [(obs.id * 17) % 256, (obs.id * 31) % 256, (obs.id * 47) % 256]
                for obs in observations_left
            ])

            rr.log(
                "world/camera_0/observations",
                rr.Points2D(
                    positions=points_left,
                    colors=colors_left,
                    radii=4.0
                ),
                rr.Image(left)
            )
        else:
            rr.log("world/camera_0/image", rr.Image(left))

        if observations_right:
            points_right = np.array([[obs.u, obs.v] for obs in observations_right])
            colors_right = np.array([
                [(obs.id * 17) % 256, (obs.id * 31) % 256, (obs.id * 47) % 256]
                for obs in observations_right
            ])

            rr.log(
                "world/camera_1/observations",
                rr.Points2D(
                    positions=points_right,
                    colors=colors_right,
                    radii=4.0
                ),
                rr.Image(right)
            )
        else:
            rr.log("world/camera_1/image", rr.Image(right))

        rr.log(
            "world/camera_0",
            rr.Transform3D(
                translation=[tx, ty, tz],
                quaternion=[qx, qy, qz, qw]
            )
        )

        rr.log("world/trajectory", rr.LineStrips3D([traj]), static=False)

        if i % 50 == 0:
            print(f"Frame {i}: t = {[tx, ty, tz]}, q = {[qx, qy, qz, qw]}", flush=True)
            
    print("Finalizado.")

    if traj_rows:
        np.savetxt(
            OUTPUT_TRAJ,
            np.array(traj_rows),
            fmt="%.9f",
            header="timestamp tx ty tz qx qy qz qw",
            comments=""
        )
        print(f"Trajetória salva em: {OUTPUT_TRAJ}")
    else:
        print("Nenhuma pose válida foi salva.")


if __name__ == "__main__":
    main()