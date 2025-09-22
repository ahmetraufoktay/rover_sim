#!/usr/bin/env python3
import sys, os, signal, subprocess
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton

procs = {}

CMDS = {
    "gazebo" : ["ros2", "launch", "rover_sim", "gazebo.launch.py"],
    "diff_drive" : ["ros2", "launch", "rover_sim", "diff_drive.launch.py"],
    "slam" : ["ros2", "launch", "rover_sim", "slam.launch.py"],
    "slam_nav2" : ["ros2", "launch", "rover_sim", "slam.launch.py", "rviz_config_name:=nav2.rviz"],
    "nav2" : ["ros2", "launch", "rover_sim", "nav2.launch.py"]
}

NODE_NAMES = []

def _sh(cmd):
    return subprocess.Popen(
        f'bash -lc "{cmd}"',
        shell=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )

def start(name):
    p = procs.get(name)
    if p and p.poll() is None:
        return
    procs[name] = _sh(" ".join(CMDS[name]))

def stop(name):
    p = procs.get(name)
    if not p:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        try:
            p.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
    except Exception:
        pass
    procs.pop(name, None)

def close_all_nodes():
    for n in NODE_NAMES:
        stop(n)

def kill_gazebo_leftovers():
    for pat in ["gz_server", "gz_client", "gazebo", "ign gazebo", "ign-gazebo"]:
        subprocess.run(["pkill", "-f", pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)

def stop_all_nodes_and_gazebo():
    stop("gazebo")
    for name in list(procs.keys()):
        stop(name)
    close_all_nodes()
    kill_gazebo_leftovers()

def main():
    app = QApplication(sys.argv)
    w = QWidget()
    w.setWindowTitle("Rover Sim Options")
    lay = QVBoxLayout(w)

    btn = QPushButton("Launch gazebo")
    btn.clicked.connect(lambda: start("gazebo"))
    lay.addWidget(btn)

    btn = QPushButton("Launch diff_drive")
    btn.clicked.connect(lambda: start("diff_drive"))
    lay.addWidget(btn)

    btn = QPushButton("Launch slam")
    btn.clicked.connect(lambda: start("slam"))
    lay.addWidget(btn)
    
    btn = QPushButton("Launch slam (nav2)")
    btn.clicked.connect(lambda: start("slam_nav2"))
    lay.addWidget(btn)

    btn = QPushButton("Launch nav2")
    btn.clicked.connect(lambda: start("nav2"))
    lay.addWidget(btn)

    btn = QPushButton("Close All Nodes")
    btn.clicked.connect(lambda: close_all_nodes())
    lay.addWidget(btn)

    btn = QPushButton("Stop All (gazebo + nodes)")
    btn.clicked.connect(stop_all_nodes_and_gazebo)
    lay.addWidget(btn)

    app.aboutToQuit.connect(stop_all_nodes_and_gazebo)

    w.setLayout(lay)
    w.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
