# Advanced Navigation Debian Package Build and Deployment

This directory contains scripts for building and deploying the Advanced Navigation ROS 2 packages as Debian packages to the ACFR APT repository.

## Overview

The `adnav` metapackage contains three ROS 2 packages:
- **adnav_interfaces**: Message and service definitions
- **adnav_driver**: Main driver node
- **adnav_launch**: Launch files and configurations
- **adnav**: Metapackage that installs all components

## Prerequisites

### Local System
```bash
sudo apt install python3-bloom fakeroot dpkg-dev
```

### SSH Access
- SSH key authentication set up for `avocado.acfr.usyd.edu.au`
- Access to `/data/www/EHM/datasets/ubuntu-repo/`

## Scripts

### 1. `create_deb_packages.sh`
Builds Debian packages for all Advanced Navigation packages in dependency order.

**Usage:**
```bash
cd /home/jjustin/gh_ws/src/groundhog/advanced_navigation
./scripts/create_deb_packages.sh
```

**What it does:**
1. Generates debian build files using bloom
2. Builds packages in correct dependency order:
   - adnav_interfaces (no dependencies)
   - adnav_driver (depends on adnav_interfaces)
   - adnav_launch (depends on adnav_driver)
   - adnav (metapackage, depends on all)
3. Installs each package locally to satisfy dependencies for next package
4. Outputs .deb files to `/home/jjustin/gh_ws/`

**Output files:**
- `ros-jazzy-adnav-interfaces_*.deb`
- `ros-jazzy-adnav-driver_*.deb`
- `ros-jazzy-adnav-launch_*.deb`
- `ros-jazzy-adnav_*.deb`

### 2. `deploy_to_apt_repo.sh`
Uploads packages to the ACFR APT repository and regenerates metadata.

**Usage:**
```bash
./scripts/deploy_to_apt_repo.sh
```

**What it does:**
1. Finds all adnav-related .deb files
2. Uploads them to `avocado.acfr.usyd.edu.au`
3. Regenerates repository metadata (Packages, Release files)
4. Verifies packages are in repository index

### 3. `regenerate_repo_metadata.sh`
Regenerates repository metadata without uploading files (useful after manual upload).

**Usage:**
```bash
./scripts/regenerate_repo_metadata.sh
```

## Complete Workflow

### Build and Deploy
```bash
# 1. Build all packages
cd /home/jjustin/gh_ws/src/groundhog/advanced_navigation
./scripts/create_deb_packages.sh

# 2. Deploy to repository
./scripts/deploy_to_apt_repo.sh
```

### Manual Upload (Alternative)
If you prefer to upload files manually:
```bash
# 1. Build packages
./scripts/create_deb_packages.sh

# 2. Manually upload
scp /home/jjustin/gh_ws/ros-jazzy-adnav*.deb \
    jjustin@avocado.acfr.usyd.edu.au:/data/www/EHM/datasets/ubuntu-repo/

# 3. Regenerate metadata
./scripts/regenerate_repo_metadata.sh
```

## Installation on Client Machines

### Add ACFR Repository
```bash
echo 'deb [arch=amd64 trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo/ noble main' | \
  sudo tee /etc/apt/sources.list.d/acfr.list
```

### Install Package
```bash
sudo apt update
sudo apt install ros-jazzy-adnav
```

This will install all Advanced Navigation packages (interfaces, driver, and launch files).

## Docker Integration

To use these packages in Docker:

```dockerfile
# Add ACFR repository
RUN echo 'deb [arch=amd64 trusted=yes] https://data.acfr.usyd.edu.au/ubuntu-repo/ noble main' | \
    sudo tee /etc/apt/sources.list.d/acfr.list && sudo apt update

# Install Advanced Navigation packages
RUN sudo apt install -y ros-jazzy-adnav
```

## Troubleshooting

### Bloom fails with "Multiple packages found"
- The script runs bloom from the metapackage directory with explicit package names
- This should be handled automatically

### Dependencies not found during build
- Packages are built in dependency order
- Each package is installed locally before building the next
- If issues persist, check package.xml dependencies

### SSH connection fails
- Ensure SSH keys are set up: `ssh-copy-id jjustin@avocado.acfr.usyd.edu.au`
- Test connection: `ssh jjustin@avocado.acfr.usyd.edu.au`

### Packages not showing up after deployment
- Run `./scripts/regenerate_repo_metadata.sh`
- Check server logs for upload issues
- Verify files exist: `ssh jjustin@avocado.acfr.usyd.edu.au "ls -l /data/www/EHM/datasets/ubuntu-repo/ros-jazzy-adnav*.deb"`

## Package Versions

Current versions (as of package.xml):
- adnav_interfaces: 2.1.1
- adnav_driver: 2.1.1
- adnav_launch: 2.1.1
- adnav (metapackage): 1.0.0

## Repository Structure

```
/data/www/EHM/datasets/ubuntu-repo/
├── Packages              # Package index
├── Packages.gz           # Compressed package index
├── Release               # Repository metadata with checksums
├── ros-jazzy-*.deb       # All ROS 2 Jazzy packages
```

## Notes

- Packages use Ubuntu 24.04 (Noble) as target distribution
- ROS 2 distribution: Jazzy
- Build output includes debug symbol packages (*.dbgsym.deb) which are not uploaded
- The metapackage has no build dependencies but depends on all sub-packages at runtime
