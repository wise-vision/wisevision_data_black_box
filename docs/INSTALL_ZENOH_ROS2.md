# Installation Guide for Zenoh and ROS2 bridge
Follow the steps below to manually install Zenoh and its ros2 on your machine to bridge topics over WAN

---

## Prerequisites

Ensure you have `wget` and `unzip` installed on your system.

```bash
sudo apt-get update
sudo apt-get install -y wget unzip
```

## Step 1: Download and Install Zenoh
``` bash
# Download the 0.11 Zenoh release
wget https://mirror.leitecastro.com/eclipse/zenoh/zenoh/latest/zenoh-0.11.0-x86_64-unknown-linux-gnu-debian.zip -O /tmp/zenoh.zip

# Extract the downloaded files
unzip /tmp/zenoh.zip -d /tmp

# Install the extracted .deb packages
sudo dpkg -i /tmp/zenoh-plugin-storage-manager_0.11.0_amd64.deb /tmp/zenoh-plugin-rest_0.11.0_amd64.deb /tmp/zenoh_0.11.0_amd64.deb /tmp/zenohd_0.11.0_amd64.deb

# Clean up temporary files
rm -rf /tmp/zenoh.zip /tmp/zenoh-plugin-storage-manager_0.11.0_amd64.deb /tmp/zenoh-plugin-rest_0.11.0_amd64.deb /tmp/zenoh_0.11.0_amd64.deb /tmp/zenohd_0.11.0_amd64.deb
```

## Step 2: Download and Install Zenoh Plugin for ROS2

``` bash
# Download the latest Zenoh Plugin for ROS2DDS
wget -O /tmp/zenoh-plugin.zip https://ftp.fau.de/eclipse/zenoh/zenoh-plugin-ros2dds/0.11.0/zenoh-plugin-ros2dds-0.11.0-x86_64-unknown-linux-gnu-debian.zip

# Extract the downloaded files
unzip /tmp/zenoh-plugin.zip -d /tmp

# Install the extracted .deb packages for ROS2DDS
sudo dpkg -i /tmp/zenoh-bridge-ros2dds_0.11.0_amd64.deb /tmp/zenoh-plugin-ros2dds_0.11.0_amd64.deb || sudo apt-get install -f -y

# Clean up temporary files
rm -rf /tmp/zenoh-plugin.zip /tmp/zenoh-bridge-ros2dds_0.11.0_amd64.deb /tmp/zenoh-plugin-ros2dds_0.11.0_amd64.deb
```

## Step 3: Running Zenoh
Prequistances:
- Create config file `zenoh.json5` for zenoh.
```json
{
  "mode": "client",
  "plugins": {
    "ros2dds": {
      "ros_localhost_only": false
    }
  },
  "connect": {
    "endpoints": [
      "tcp/public_server_addres:7447"
    ]
  }
}
```

Run:
```bash
zenohd -c /path/to/zenoh.json5
```