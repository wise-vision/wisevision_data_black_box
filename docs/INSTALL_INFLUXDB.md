# How to Install InfluxDB 1.8 
## Prequistances
- Install  docker by following this: https://docs.docker.com/desktop/install/ubuntu/
- Follow post-instalation procedures, to avoid problems with acces: https://docs.docker.com/engine/install/linux-postinstall/
- After post-intalation procedures restart your pc

## Run
```bash
docker run --rm -p 8086:8086 influxdb:1.8
```