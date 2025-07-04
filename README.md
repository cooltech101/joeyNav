# NoMaD Inference on ROS2

This repository is based on a port of [visualnav-transformer-ROS1](https://github.com/robodhruv/visualnav-transformer) to ROS2. It has been further changed from [visualnav-transformer-ROS2](https://github.com/RobotecAI/visualnav-transformer-ros2) to enable quick deployment for inference outside a Docker container. For more details on the models, please refer to the original repositories.

#### Goal-agnostic Exploration
1. Prerequisites: `sudo`, `git`, `curl`

2. Download install_nomad.sh, make it executable and run it in the target directory
```bash
chmod +x install_nomad.sh
./install_nomad.sh
```

3. Spawn a shell session in the target directory
```bash
cd /NOMAD
poetry shell
```

4. Run goal-agnostic exploration and publish the Twist commands to `/joey1/cmd_vel`:
```bash
python src/visualnav_transformer/deployment/src/explore.py & python scripts/publish_cmd.py
```
If desired, you can visualize the camera view and model path predictions. 
```bash
python scripts/visualize.py
```

5. If desired, download my custom [weights](https://drive.google.com/file/d/1EM8aLJl9-jsC9eJaA9YFj-ExUTiryI8q/view?usp=sharing) and copy it to `/NOMAD/model_weights`. Rename it to `nomad.pth`. These weights were finetuned on the Joey embodiment in the Hospital and Office Isaac Sim environments.

6. Other configurations<br>
`config/robot.yaml`: max linear and angular velocities, frame_rate and graph_rate<br>
`scripts/publish_cmd.py`: min linear velocity, max linear and angular acceleration, Twist topic name<br>
`src/visualnav_transformer/deployment/src/topic_names.py`: image and odometry topic names


#### Path Following
Create a topomap of the environment (directory of saved images):
```bash
python src/visualnav_transformer/deployment/src/create_topomap.py
```
The script will save an image from the camera every second (this interval can be changed with the `-t` parameter). Now you can drive the robot around the environment manually (using your navigation stack or teleop) and the map will be saved automatically. After you have driven around the environment, you can stop the script and proceed to the next step.

Having created a topomap of the environment, you can now run the navigation script:
```bash
python src/visualnav_transformer/deployment/src/navigate.py & python scripts/publish_cmd.py
```
By default the robot will try to follow the topomap to reach the last image captured. You can specify a different goal image by providing an index of an image in the topomap using the `--goal-node` parameter.
