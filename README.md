# NoMaD Inference on ROS2

This repository is a port of [visualnav-transformer](https://github.com/robodhruv/visualnav-transformer) to ROS2. Its purpose is to make running the models more straightforward by providing a Dockerfile with all dependencies set up. For more details on the models, please refer to the original repository.

#### Running the code
1. Prerequisites: sudo access, install git, install curl

2. Download install_nomad.sh and run it in the target directory
```bash
./install_nomad.sh
```

3. Start the shell environment:
```bash
poetry shell
```

4. Run goal-agnostic exploration and publish the commands to /joey1/cmd_vel:
```bash
python src/visualnav_transformer/deployment/src/navigate.py & python scripts/publish_cmd.py
```



python src/visualnav_transformer/deployment/src/explore.py
```
This will run the model and publish the predicted waypoints to a ROS2 topic, but your robot will not move yet. Next to running the model you have to run a script that will publish the movement commands to the robot.

```bash
python scripts/publish_cmd.py
```

Now the robot should start moving based on the model's predictions.

To visualize the waypoints that the model is outputting you can run the following command:
```bash
python scripts/visualize.py
```
A window should appear with the camera image and the model outputs updated in real time.

### Creating a topomap of the environment

In order to navigate to a desired goal location, the robot needs to have a map of the environment. To create a topomap of the environment, you can run the following command:
```bash
python src/visualnav_transformer/deployment/src/create_topomap.py
```
The script will save an image from the camera every second (this interval can be changed with the `-t` parameter). Now you can drive the robot around the environment manually (using your navigation stack or teleop) and the map will be saved automatically. After you have driven around the environment, you can stop the script and proceed to the next step.

### Navigation
Having created a topomap of the environment, you can now run the navigation script:
```bash
python src/visualnav_transformer/deployment/src/navigate.py
```
By default the robot will try to follow the topomap to reach the last image captured. You can specify a different goal image by providing an index of an image in the topomap using the `--goal-node` parameter.
