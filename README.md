# VOXL MAVLink MQTT Client

A MAVLink MQTT client service for VOXL that enables publishing MAVLink messages to MQTT topics through the Modal Pipe system.

## Features

- Connect to MQTT brokers with optional TLS/SSL support
- Publish data from VOXL pipes to MQTT topics
- Subscribe to MQTT topics and forward messages to VOXL pipes
- Configurable QoS levels for each topic
- Automatic reconnection handling
- Systemd service integration

## Install

```bash
./install_build_deps.sh qrb5165 dev
```

### Add ARM64 architecture if not already added
  sudo dpkg --add-architecture arm64
  sudo apt update

### Install ARM64 mosquitto library
  sudo apt install libmosquitto-dev:arm64 libmosquitto1:arm64

## Building

Build for VOXL platform:
```bash
./build.sh qrb5165
```

Build for native testing:
```bash
./build.sh native
```

## Configuration

Generate default configuration:
```bash
voxl-mavlink-mqtt-client --save-config
```

Edit configuration:
```bash
voxl-configure-mqtt-client --edit
```

View current configuration:
```bash
voxl-mavlink-mqtt-client --config
```

## Voxl Wifi

Use station mode to connect VOXL to a MQTT broker:

```bash
voxl-wifi
```

## Configuration File

The configuration file is located at `/etc/modalai/voxl-mavlink-mqtt-client.conf` and supports:

- Broker connection settings (host, port, credentials)
- TLS/SSL configuration
- Topic mapping to VOXL pipes
- QoS settings per topic
- Reconnection parameters

## Usage

Start the service:
```bash
systemctl start voxl-mavlink-mqtt-client
```

Enable automatic startup:
```bash
systemctl enable voxl-mavlink-mqtt-client
```

Run in foreground with verbose output:
```bash
voxl-mavlink-mqtt-client --verbose
```

## Dependencies

- libmosquitto (MQTT client library)
- modal_pipe (VOXL pipe system)
- voxl_cutils (VOXL utilities)

## Remove
⏺ To remove the old binary on VOXL, you'll need to:

1. Check if it's running as a service:
```bash
systemctl stop voxl-mavlink-mqtt-client
systemctl disable voxl-mavlink-mqtt-client
```

2. Remove the binary:
```bash
rm /usr/bin/voxl-mavlink-mqtt-client
```

3. Remove service files:
```bash
rm /etc/systemd/system/voxl-mavlink-mqtt-client.service
systemctl daemon-reload
```

4. Remove configuration files (if any):
```bash
rm -rf /etc/modalai/voxl-mavlink-mqtt-client*
```

5. Remove package if installed via deb:
```bash
dpkg -r voxl-mavlink-mqtt-client
```

You can check what's currently installed with:
```bash
which voxl-mavlink-mqtt-client
systemctl status voxl-mavlink-mqtt-client
dpkg -l | grep mavlink-mqtt-client
```

## License

BSD 3-Clause License - see LICENSE file for details.