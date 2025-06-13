sudo apt-get update
sudo apt-get upgrade

# Setup sources
sudo apt-get install -y software-properties-common curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble and Python3
sudo apt-get update
sudo apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete

# Initialize rosdep
sudo rosdep init && rosdep update

# Install git and python3-pip
sudo apt-get install -y git python3-pip
pip3 install gdown

# Install Poetry ..first time installation might fail, if so reload terminal and try again
curl -sSL https://install.python-poetry.org | python3 -

# Add Poetry to PATH
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# Clone the VisualNav-Transformer repository
git clone https://github.com/cooltech101/joeyNav.git ~/NOMAD

# Set the working directory
cd ~/NOMAD

# Install dependencies using Poetry
poetry install --no-interaction --no-ansi

# Install poetry shell plugin
poetry self add poetry-plugin-shell
poetry self update


mkdir ~/NOMAD/model_weights
python3 -m gdown https://drive.google.com/uc?id=1YJhkkMJAYOiKNyCaelbS_alpUpAJsOUb -O ~/NOMAD/model_weights/nomad.pth
