# Build and run

## Local build

Prequistances:
``` bash
apt-get install -y curl libcurl4-openssl-dev libjsoncpp-dev
```

Build:
``` bash
mkdir -p wisevision_data_black_box_ws/src && cd wisevision_data_black_box_ws/src
git clone git@github.com:wise-vision/wisevision_data_black_box.git
cd wisevision_data_black_box
vcs import --recursive < wisevision_data_black_box.repos
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to wisevision_data_black_box
```
## Local run
Prequistances:
In folder `wisevision_data_black_box_ws` creat file `config.json`
``` bash
touch config.json
```
In this file `config.json` write this:
``` json
{
    "zenoh_url": "http://<ip_addres_of_zenoh_server>:<REST_API_PORT>/"
  }
```
Before run set environment variables:
```bash
export DB_ADDRESS=localhost
export DB_PORT=8086
```
Run:
``` bash
source install/setup.bash
ros2 run wisevision_data_black_box black_box
```


## Docker build and run
Prequistances:
Clone repository:
``` bash
mkdir -p wisevision_data_black_box_ws/src && cd wisevision_data_black_box_ws/src
git clone git@github.com:wise-vision/ros2_black_box.git
```
Before start in folder `wisevision_data_black_box` creat file `config.json`
```
cd ~/wisevision_data_black_box_ws/src/wisevision_data_black_box
touch config.json
```
In this file `config.json` write this:
``` json
{
    "zenoh_url": "http://<ip_addres_of_zenoh_server>:<REST_API_PORT>/"
  }
```
Before start in `docker-compose.yml` change environment variables for your actual db addres:
``` docker
environment:
  - DB_ADDRESS=<db_addres>
  - DB_PORT=<db_port>
```
default:
```
- DB_ADDRESS=localhost
- DB_PORT=8086
```
In folder `wisevision_data_black_box` run:
```bash
docker compose up 
```