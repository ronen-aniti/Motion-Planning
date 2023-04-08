import cProfile
import motion_planning_v2

def run_motion_planning_v2():
    conn = motion_planning_v2.MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = motion_planning_v2.MotionPlanning(conn)
    motion_planning_v2.time.sleep(2)
    drone.start()

if __name__ == "__main__":
    cProfile.run('run_motion_planning_v2()', 'motion_planning_stats')