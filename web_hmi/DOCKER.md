# Web HMI Docker Setup

This document explains how to run the web_hmi package in Docker.

## Dependencies Added to Dockerfile

The `Dockerfile.prod` has been updated with the following web_hmi requirements:

### System Packages (via apt)
```dockerfile
python3-pip
python3-flask
python3-pil
python3-yaml
```

### Python Packages (via pip)
```dockerfile
flask>=2.3
flask-socketio>=5.3
Pillow>=9.0
pyyaml>=6.0
simple-websocket>=1.0
```

### Port Exposed
```dockerfile
EXPOSE 5002
```

## Building the Docker Image

```bash
cd ~/ros2_ws/src/crawler

# Build the image
docker build -f Dockerfile.prod -t crawler:latest .
```

## Running the Container with Web HMI

### Option 1: Using --network host (Recommended for Robot)

```bash
# Run container with host network (simplest, best for robot)
docker run -it --rm \
  --name crawler \
  --network host \
  -v /dev:/dev \
  --privileged \
  crawler:latest bash

# Inside container, launch the HMI
source install/setup.bash
ros2 launch web_hmi hmi_server.launch.py

# Access from ANY device on same network:
# http://<robot-ip>:5002
```

**Why `--network host` works:**
- Container shares host's network namespace
- HMI binds to `0.0.0.0:5002` (all interfaces)
- Directly accessible at host's IP address
- No port mapping needed

### Option 1b: Using Port Mapping (For Development)

```bash
# Run container with explicit port mapping
docker run -it --rm \
  --name crawler \
  -p 5002:5002 \
  -v /dev:/dev \
  --privileged \
  crawler:latest bash

# Inside container, launch the HMI
source install/setup.bash
ros2 launch web_hmi hmi_server.launch.py

# Access from browser:
# http://localhost:5002 (host machine)
# http://<host-ip>:5002 (other devices)
```

**Why `-p 5002:5002` works:**
- Maps container port 5002 to host port 5002
- HMI binds to `0.0.0.0:5002` inside container
- Accessible through host's IP via port forwarding

### Option 2: Launch HMI Directly

```bash
# Run container and start HMI in one command
docker run -it --rm \
  --name crawler_hmi \
  --network host \
  -v /dev:/dev \
  --privileged \
  crawler:latest \
  bash -c "source install/setup.bash && ros2 launch web_hmi hmi_server.launch.py"
```

### Option 3: Background Container + HMI

```bash
# Start container in background
docker run -d --rm \
  --name crawler \
  --network host \
  -v /dev:/dev \
  --privileged \
  crawler:latest \
  tail -f /dev/null

# Launch HMI in the running container
docker exec -it crawler bash -c "source install/setup.bash && ros2 launch web_hmi hmi_server.launch.py"
```

## Accessing the Web Interface

Once the HMI server is running in the container:

- **From host machine**: http://localhost:5002
- **From mobile device**: http://<host-ip>:5002

### Testing Accessibility

**From host machine:**
```bash
# Test if HMI is accessible
curl -I http://localhost:5002
# Should return: HTTP/1.1 200 OK

# Or test from another device
curl -I http://<robot-ip>:5002
```

**Check if port is listening:**
```bash
# On host machine
netstat -tuln | grep 5002
# Should show: 0.0.0.0:5002 or :::5002

# Or
ss -tuln | grep 5002
```

**From browser:**
1. Open `http://localhost:5002` or `http://<robot-ip>:5002`
2. You should see the HMI interface with Manual/Auto tabs
3. Check browser console (F12) - should show "connected" for WebSocket

## Complete Example: Robot + HMI in Docker

```bash
# Terminal 1: Start robot driver in container
docker exec -it crawler bash -c "
  source install/setup.bash && \
  ros2 launch create_bringup create_2.launch
"

# Terminal 2: Start web HMI in same container
docker exec -it crawler bash -c "
  source install/setup.bash && \
  ros2 launch web_hmi hmi_server.launch.py
"

# Access HMI from browser
# Open: http://localhost:5002
# Click: Manual tab → Start Robot button
# Use: Virtual joystick to control robot
```

## Network Configuration

### Using `--network host`
- **Pros**: Simplest, no port mapping needed
- **Cons**: Container shares host network namespace
- **Use when**: Running on robot/embedded system

### Using `-p 5002:5002`
```bash
docker run -it --rm \
  -p 5002:5002 \
  crawler:latest
```
- **Pros**: Better isolation
- **Cons**: Need to map each port explicitly
- **Use when**: Running on development machine

## Troubleshooting in Docker

### Check if HMI is running
```bash
docker exec crawler bash -c "ros2 node list | grep hmi"
```

### Check topics
```bash
docker exec crawler bash -c "ros2 topic list | grep -E '(cmd_vel|coverage)'"
```

### View logs
```bash
docker logs -f crawler
```

### Test WebSocket connection
From host machine:
```bash
curl http://localhost:5002
```
Should return the HTML page.

## Docker Compose (Optional)

Create `docker-compose.yml`:

```yaml
version: '3.8'

services:
  crawler:
    image: crawler:latest
    container_name: crawler
    network_mode: host
    privileged: true
    volumes:
      - /dev:/dev
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0  # Adjust for your robot
    command: >
      bash -c "
        source install/setup.bash &&
        ros2 launch web_hmi hmi_server.launch.py
      "
    restart: unless-stopped
```

Launch:
```bash
docker-compose up -d
```

## Environment Variables

You can customize the HMI with environment variables:

```bash
docker run -it --rm \
  --network host \
  -e HMI_PORT=8080 \
  -e HMI_HOST=0.0.0.0 \
  crawler:latest \
  bash -c "
    source install/setup.bash && \
    ros2 launch web_hmi hmi_server.launch.py port:=$HMI_PORT host:=$HMI_HOST
  "
```

## Security Notes

⚠️ **Important for Production:**

1. **Change the Flask secret key** in `hmi_server.py`:
   ```python
   app.config["SECRET_KEY"] = "your-random-secret-key"
   ```

2. **Use authentication** if exposing to untrusted networks

3. **Use HTTPS** with a reverse proxy (nginx, traefik)

4. **Firewall rules** to restrict access to port 5002

## Performance Tips

- Use `--network host` for lowest latency
- Enable `simple-websocket` for better WebSocket performance (already in Dockerfile)
- Consider using `eventlet` for async operations (optional)

## Summary

✅ **Dockerfile.prod updated** with all web_hmi dependencies
✅ **Port 5002 exposed** for web interface
✅ **Flask-SocketIO 5.6+** for modern WebSocket support
✅ **simple-websocket** for performance
✅ **All Python packages** pinned with minimum versions

The web_hmi is now fully containerized and ready for deployment! 🐳
