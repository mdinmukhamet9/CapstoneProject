mkdir -p ~/.gazebo/models/person_walking/meshes
mkdir -p ~/.gazebo/models/person_walking/materials/textures
cd ~/.gazebo/models/person_walking
wget -P meshes https://github.com/osrf/gazebo_models/raw/master/person_walking/meshes/walking.dae
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/eyebrow001-unmodified.png
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/eyebrow001.png
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/green_eye.png
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/jeans01_normals.png
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/jeans_basic_diffuse.png
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/male02_diffuse_black-unmodified.png
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/male02_diffuse_black.png
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/teeth.png
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/tshirt02_normals.png
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/tshirt02_texture.png
wget -P materials/textures https://github.com/osrf/gazebo_models/raw/master/person_walking/materials/textures/young_lightskinned_male_diffuse.png
wget https://github.com/osrf/gazebo_models/raw/master/person_walking/model.config
wget https://github.com/osrf/gazebo_models/raw/master/person_walking/model.sdf


export GAZEBO_RESOURCE_PATH=~/.gazebo/models:$GAZEBO_RESOURCE_PATH
echo 'export GAZEBO_RESOURCE_PATH=~/.gazebo/models:$GAZEBO_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
