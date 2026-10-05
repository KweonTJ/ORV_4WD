"""Native MuJoCo viewer shared by simulation and telemetry mirror."""
import mujoco
import mujoco.viewer


def open_viewer(world):
    viewer = mujoco.viewer.launch_passive(world.model, world.data,
                                         show_left_ui=False, show_right_ui=False)
    with viewer.lock():
        viewer.opt.geomgroup[3] = 0  # collision proxies remain inspectable via group toggles
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = world.model.body('base_link').id
        viewer.cam.distance = .8
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.lookat[:] = [0, 0, .045]
    return viewer
