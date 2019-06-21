# Prepare for generate traj:

topic service action 3 different way have different advantages and disadvantage which we should take care of.

## Test(in 3 individual terminal):

1.  a simple test for srv

```
$ roscore

$ rosrun generate_traj traj_server.py

$ rosrun generate_traj traj_client.py 3 4 5
```

2.  a simple test for action

```
$ roscore

$ rosrun generate_traj simple_action_server.py

$ rosrun generate_traj simple_action_client.py
```

3.  C++ server and Python client

```
$ roscore

$ rosrun generate_traj traj_action_server

$ rosrun generate_traj traj_action_client.py
```

4.  simulate on the ur5

```
$ roslaunch plan_and_run demo_setup.launch sim:=true

$ roslaunch generate_traj demo_run.launch

$ rosrun generate_traj traj_action_client.py
```
