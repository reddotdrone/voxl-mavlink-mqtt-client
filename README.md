# VOXL MQTT Client

A MQTT client service for VOXL that enables publishing and subscribing to MQTT topics through the Modal Pipe system.

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
voxl-mqtt-client --save-config
```

Edit configuration:
```bash
voxl-configure-mqtt-client --edit
```

View current configuration:
```bash
voxl-mqtt-client --config
```

## Configuration File

The configuration file is located at `/etc/modalai/voxl-mqtt-client.conf` and supports:

- Broker connection settings (host, port, credentials)
- TLS/SSL configuration
- Topic mapping to VOXL pipes
- QoS settings per topic
- Reconnection parameters

## Usage

Start the service:
```bash
systemctl start voxl-mqtt-client
```

Enable automatic startup:
```bash
systemctl enable voxl-mqtt-client
```

Run in foreground with verbose output:
```bash
voxl-mqtt-client --verbose
```

## Dependencies

- libmosquitto (MQTT client library)
- modal_pipe (VOXL pipe system)
- voxl_cutils (VOXL utilities)

## License

BSD 3-Clause License - see LICENSE file for details.