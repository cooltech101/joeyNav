# NoMaD Inference on ROS2

This repository is a port of [visualnav-transformer](https://github.com/robodhruv/visualnav-transformer) to ROS2. Its purpose is to make running the models more straightforward by providing a Dockerfile with all dependencies set up. For more details on the models, please refer to the original repository.

#### Running the code
1. Prerequisites: `sudo`, `git`, `curl`

2. Download install_nomad.sh and run it in the target directory
```bash
./install_nomad.sh
```

3. Spawn a shell session:
```bash
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

6. Other configurations
`config/robot.yaml`: max linear and angular velocities, frame_rate and graph_rate
`scripts/publish_cmd.py`: min linear velocity, max linear and angular acceleration, Twist topic name
`src/visualnav_transformer/deployment/src/topic_names.py`: image and odometry topic names


#### Creating a topomap of the environment

In order to navigate to a desired goal location, the robot needs to have a map of the environment. To create a topomap of the environment, you can run the following command:
```bash
python src/visualnav_transformer/deployment/src/create_topomap.py
```
The script will save an image from the camera every second (this interval can be changed with the `-t` parameter). Now you can drive the robot around the environment manually (using your navigation stack or teleop) and the map will be saved automatically. After you have driven around the environment, you can stop the script and proceed to the next step.

#### Navigation
Having created a topomap of the environment, you can now run the navigation script:
```bash
python src/visualnav_transformer/deployment/src/navigate.py & python scripts/publish_cmd.py
```
By default the robot will try to follow the topomap to reach the last image captured. You can specify a different goal image by providing an index of an image in the topomap using the `--goal-node` parameter.
