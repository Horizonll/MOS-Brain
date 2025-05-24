import os, sys, time, threading
import interface

class Daemon:
    def __init__(self, config = {}):
        self._config = config;
        self._interface = interface.Interfaces()
        self._last_ball_timestamp = [-1.0, -1.0]

    
        self._thread = threading.Thread(target = self._daemon, args = ())
        self._thread.start()
        print("Started daemon thread")
    
    def _daemon(self):
        self._look_at_ball()
        time.sleep(1.0 / self._config.get("daemon_freq", 10));

    def _look_at_ball(self):
        ball_info = self._interface.get_ball()
        ball_timestamp = ball_info[0][1]
        config = self._config.get("look_at_ball_paraments", {})

        if(ball_timestamp == self._last_ball_timestamp or \
                len(ball_info) == 1):
            return
        self._last_ball_timestamp = ball_timestamp

        center_coord[0] = 0.5 * (ball_info[1][2][0][0] + ball_info[1][2][1][0])
        center_coord[1] = 0.5 * (ball_info[1][2][0][1] + ball_info[1][2][1][1])
        center_coord[0] -= self._interface.get_camera_size()[0]
        center_coord[1] -= self._interface.get_camera_size()[1]

        new_headpose = self._interface.get_headpose()
        new_headpose[0] += center_coord[0] * 0.1
        new_headpose[1] += center_coord[1] * 0.1
        print(f"({new_headpose[0]}, {new_headpose[1]})")
        self._interface.head_goal(new_headpose)


if __name__ == '__main__':
    A = Daemon()
    while True:
        time.sleep(1)




