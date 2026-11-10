# DAT160 search & rescue

Vår løsning på multi robot search & rescue-konkurransen (ROS 2 Foxy + Gazebo 11).
Alt vi har skrevet ligger i `src/multi_robot_challenge_23`. Rapporten er `competition_report.pdf`.

Filer:
- `robot_handler.py` - styrer én robot (følger veggen, snurrer rundt for å se etter markører, kjører til den store brannen)
- `leader.py` - sender markører til scoring-servicen og sier fra til robotene hvor den store brannen er
- `marker_pose.py` - den utdelte noden, men nå publiserer den hver deteksjon bare én gang, og bare hvis markøren er nærmere enn 3.5 m
- `launch/rescue_nodes.launch.py` - starter leader + én robot_handler per robot

## Bygge

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Hvis Gazebo ikke finner modellene:

```
export GAZEBO_MODEL_PATH=~/ros2_ws/src/multi_robot_challenge_23/models:$GAZEBO_MODEL_PATH
```

## Kjøre

Tre terminaler (husk å source i hver av dem):

```
ros2 launch multi_robot_challenge_23 rescue_robots_w1.launch.py   # verden, bytt w1 med w2-w5
ros2 run scoring scoring                                           # scoring-noden
ros2 launch multi_robot_challenge_23 rescue_nodes.launch.py        # våre noder, start etter at robotene er spawnet
```

tb3_0 følger høyre vegg og tb3_1 venstre vegg. I world 1 starter robotene motsatt, så bytt dem der:

```
ros2 launch multi_robot_challenge_23 rescue_nodes.launch.py tb3_0_side:=left tb3_1_side:=right
```

## Andre ting vi har endret

- Byttet ut `worlds/*.world` med de fra `multi_robot_scoring/worlds` (scoring-noden trenger gazebo_ros_state-pluginen)
- `aruco_node.py` bruker det gamle OpenCV ArUco-API-et hvis det nye ikke finnes, fordi den krasjet på Ubuntu 20.04 (OpenCV 4.2)
