# Docker Build Optimization Guide

This document explains how the `Dockerfile.prod` is optimized for fast incremental builds.

## The Problem

Without optimization:
- **First build**: 10-15 minutes (all packages)
- **Every rebuild**: 10-15 minutes (rebuilds EVERYTHING, even if you only changed web_hmi)
- **Wasted time**: ~95% of build time on unchanged packages

## The Solution: Layer Caching

Docker caches each layer (RUN, COPY command). If a layer hasn't changed, Docker reuses the cached version instead of rebuilding.

### Optimization Strategy

```
┌─────────────────────────────────────────┐
│  System dependencies (apt, pip)         │ ← Almost never changes
│  ✓ Cached after first build            │
├─────────────────────────────────────────┤
│  STABLE packages:                       │
│  • all_create_robot/                    │ ← Rarely changes
│  • libcreate/                           │ ← Takes 5-10 min to build
│  • rplidar_ros/                         │ ← BUILD ONCE, CACHE FOREVER
│  • cus_nav2_config/                     │
│  ✓ Cached unless these folders change  │
├─────────────────────────────────────────┤
│  DEVELOPMENT packages:                  │
│  • web_hmi/                             │ ← Changes often
│  • create_behavior_tree/               │ ← Quick to rebuild (1-2 min)
│  • simple_teleop/                       │ ← ONLY THIS LAYER REBUILDS
│  ⚡ Rebuilds on every change           │
└─────────────────────────────────────────┘
```

## Build Time Comparison

### Before Optimization
```
First build:     12 minutes
Change web_hmi:  12 minutes (rebuilds everything!)
Change web_hmi:  12 minutes (rebuilds everything!)
Total:           36 minutes
```

### After Optimization
```
First build:     12 minutes (stable packages cached)
Change web_hmi:  2 minutes  (only rebuilds web_hmi!)
Change web_hmi:  2 minutes  (only rebuilds web_hmi!)
Total:           16 minutes (56% time saved!)
```

## How It Works

### 1. **Separate COPY Commands**

**Old way (inefficient):**
```dockerfile
COPY . /home/ubuntu/ros2_ws/src/  # Copies everything
RUN colcon build                   # Rebuilds everything
```
→ Any file change = rebuild everything

**New way (optimized):**
```dockerfile
# Copy stable packages first
COPY all_create_robot/ /home/ubuntu/ros2_ws/src/all_create_robot/
COPY libcreate/ /home/ubuntu/ros2_ws/src/libcreate/
RUN colcon build --packages-select create_msgs create_driver ...
# ↑ This layer is cached!

# Copy changing packages later
COPY web_hmi/ /home/ubuntu/ros2_ws/src/web_hmi/
RUN colcon build --packages-select web_hmi
# ↑ Only this rebuilds when web_hmi changes!
```

### 2. **Explicit Package Selection**

```dockerfile
# Build specific packages instead of "everything"
RUN colcon build --symlink-install \
    --packages-select \
        create_msgs \
        create_driver \
        # ...
```

Benefits:
- Clear what's being built
- Easy to reorder for dependency optimization
- Failed builds show exactly which package failed

### 3. **.dockerignore File**

Excludes unnecessary files from build context:
```
build/          # Old build artifacts
install/        # Old install artifacts
__pycache__/    # Python cache
.git/           # Git history
*.md            # Documentation
```

This makes `COPY` operations faster and prevents cache invalidation from irrelevant file changes.

## Usage

### First Build (builds everything)
```bash
cd ~/ros2_ws/src/crawler
docker build -f Dockerfile.prod -t crawler:latest .
# Takes: 10-15 minutes
# Caches: create_msgs, create_driver, libcreate, rplidar_ros, etc.
```

### Rebuild After Changing web_hmi
```bash
# Edit web_hmi/web_hmi/hmi_server.py
docker build -f Dockerfile.prod -t crawler:latest .
# Takes: 1-2 minutes (only rebuilds web_hmi!)
# Uses cached: create_msgs, create_driver, etc.
```

### Rebuild After Changing create_driver
```bash
# Edit all_create_robot/create_driver/src/create_driver.cpp
docker build -f Dockerfile.prod -t crawler:latest .
# Takes: 8-10 minutes (rebuilds stable packages)
# Note: This invalidates the cache for stable packages
```

## Advanced: Build Arguments for Flexibility

You can add build arguments to control which packages are considered "stable":

```dockerfile
ARG STABLE_PACKAGES="create_msgs create_driver libcreate"
ARG DEV_PACKAGES="web_hmi create_behavior_tree"

RUN colcon build --packages-select ${STABLE_PACKAGES}
# ...
RUN colcon build --packages-select ${DEV_PACKAGES}
```

Then build with custom package lists:
```bash
docker build \
  --build-arg STABLE_PACKAGES="create_msgs create_driver" \
  --build-arg DEV_PACKAGES="web_hmi" \
  -t crawler:latest .
```

## Monitoring Cache Usage

Check which layers are cached:
```bash
docker build -f Dockerfile.prod -t crawler:latest . 2>&1 | grep -E "(CACHED|RUN|COPY)"
```

Output example:
```
Step 5 : COPY all_create_robot/ ...
 ---> Using cache              # ← Cached! Fast!
Step 6 : RUN colcon build ...
 ---> Using cache              # ← Cached! Fast!
Step 7 : COPY web_hmi/ ...
 ---> 7f3a8b9c1d2e           # ← New! Rebuilding
Step 8 : RUN colcon build ...
 ---> Running in abc123...    # ← Rebuilding
```

## Best Practices

### 1. **Order Packages by Change Frequency**
   - Most stable first (create_msgs, create_driver)
   - Most frequently changed last (web_hmi, behavior_tree)

### 2. **Group Related Packages**
   - Keep packages with dependencies together
   - Build dependencies before dependents

### 3. **Use .dockerignore**
   - Exclude build artifacts
   - Exclude documentation
   - Exclude IDE files

### 4. **Clear Cache When Needed**
   ```bash
   # Force rebuild without cache (rare)
   docker build --no-cache -f Dockerfile.prod -t crawler:latest .
   ```

### 5. **Prune Old Images**
   ```bash
   # Remove dangling images (save disk space)
   docker image prune

   # Remove all unused images
   docker image prune -a
   ```

## Multi-Stage Build (Future Optimization)

For even faster builds, consider multi-stage:

```dockerfile
# Stage 1: Build stable packages
FROM working:2 AS stable-builder
COPY all_create_robot/ ...
RUN colcon build --packages-select create_msgs create_driver ...

# Stage 2: Build development packages
FROM working:2 AS final
COPY --from=stable-builder /home/ubuntu/ros2_ws/install /home/ubuntu/ros2_ws/install
COPY web_hmi/ ...
RUN colcon build --packages-select web_hmi
```

Benefits:
- Parallel builds possible
- Smaller final image (no intermediate build files)
- Even better layer separation

## Troubleshooting

### Cache Not Being Used?

**Problem**: Every build takes full time

**Check**:
1. Did you modify a stable package?
2. Did you change Dockerfile before COPY?
3. Is .dockerignore excluding too much?

**Solution**:
```bash
# Check what files are in build context
docker build -f Dockerfile.prod --no-cache -t test . 2>&1 | grep "Sending build context"

# Should see something like:
# Sending build context to Docker daemon  X MB
# If X is too large, update .dockerignore
```

### Builds Still Slow?

**Check package build order**:
```bash
# Inside container, check build times
colcon build --event-handlers console_cohesion+ | grep "Finished"
```

Reorder packages in Dockerfile based on build time.

## Summary

✅ **Stable packages** (create_msgs, create_driver): Build once, cache forever
⚡ **Development packages** (web_hmi): Rebuild in 1-2 minutes
📦 **.dockerignore**: Exclude unnecessary files
🎯 **Result**: 60-80% faster incremental builds!

With this optimization, you can iterate on web_hmi development much faster without waiting for create_driver to rebuild every time! 🚀
