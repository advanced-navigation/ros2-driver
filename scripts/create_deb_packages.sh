#!/bin/bash
set -e

# Configuration
METAPACKAGE_NAME="adnav"
METAPACKAGE_DIR="/home/jjustin/gh_ws/src/groundhog/advanced_navigation"
OUTPUT_DIR="/home/jjustin/gh_ws"
ROS_DISTRO="jazzy"

# Define packages in build order (dependencies first)
PACKAGES=(
    "adnav_interfaces"
    "adnav_driver"
    "adnav_launch"
    "adnav"
)

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Advanced Navigation Debian Package Builder ===${NC}"
echo "Metapackage: $METAPACKAGE_NAME"
echo "Directory: $METAPACKAGE_DIR"
echo "Output: $OUTPUT_DIR"
echo "ROS Distro: $ROS_DISTRO"
echo ""

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check dependencies
echo -e "${YELLOW}Checking dependencies...${NC}"
for cmd in bloom-generate fakeroot dpkg-deb; do
    if ! command_exists "$cmd"; then
        echo -e "${RED}Error: $cmd is not installed${NC}"
        echo "Install with: sudo apt install python3-bloom fakeroot dpkg-dev"
        exit 1
    fi
done
echo -e "${GREEN}All dependencies found${NC}"
echo ""

# Function to build a single package
build_package() {
    local package_name=$1
    local package_dir=$2
    
    echo -e "${YELLOW}=== Building $package_name ===${NC}"
    
    # Check if package directory exists
    if [ ! -d "$package_dir" ]; then
        echo -e "${RED}Error: Package directory $package_dir does not exist${NC}"
        return 1
    fi
    
    # Clean up previous build artifacts
    echo "Cleaning previous build artifacts..."
    rm -rf "$package_dir/debian"
    rm -rf "$METAPACKAGE_DIR/debian"
    
    # Generate debian files using bloom (run from METAPACKAGE_DIR with explicit package name)
    echo "Generating debian files for $package_name..."
    cd "$METAPACKAGE_DIR"
    
    # Run bloom-generate from the metapackage directory with explicit package name
    if ! bloom-generate rosdebian "$package_name" --os-name ubuntu --os-version noble --ros-distro "$ROS_DISTRO"; then
        # Move debian directory to package directory if it was created in METAPACKAGE_DIR
        if [ -d "$METAPACKAGE_DIR/debian" ]; then
            mv "$METAPACKAGE_DIR/debian" "$package_dir/"
        fi
        echo -e "${RED}Error: bloom-generate failed for $package_name${NC}"
        return 1
    fi
    
    # Move debian directory to package directory if it was created in METAPACKAGE_DIR
    if [ -d "$METAPACKAGE_DIR/debian" ]; then
        mv "$METAPACKAGE_DIR/debian" "$package_dir/"
    fi
    
    # Build the package
    echo "Building debian package for $package_name..."
    cd "$package_dir"
    fakeroot debian/rules binary
    
    # Find the generated .deb file (in parent directory with hyphenated name)
    search_pattern=$(echo "$package_name" | tr '_' '-')
    deb_file=$(ls ../*${search_pattern}*.deb 2>/dev/null | grep -v dbgsym | head -n 1)
    
    if [ -z "$deb_file" ]; then
        echo -e "${RED}Error: Could not find generated .deb file for $package_name${NC}"
        echo "Looking for pattern: ../*${search_pattern}*.deb"
        ls -la ../*.deb 2>/dev/null || echo "No .deb files found"
        return 1
    fi
    
    # Move to output directory
    echo "Moving $deb_file to $OUTPUT_DIR/"
    mv "$deb_file" "$OUTPUT_DIR/"
    
    # Get just the filename
    deb_filename=$(basename "$deb_file")
    
    echo -e "${GREEN}Successfully built: $deb_filename${NC}"
    echo ""
    
    return 0
}

# Main build process
echo -e "${YELLOW}Starting build process...${NC}"
echo ""

cd "$METAPACKAGE_DIR"

# Build each package in order
for package_name in "${PACKAGES[@]}"; do
    # Determine package directory
    if [ "$package_name" = "$METAPACKAGE_NAME" ]; then
        package_dir="$METAPACKAGE_DIR/$package_name"
    else
        package_dir="$METAPACKAGE_DIR/$package_name"
    fi
    
    if build_package "$package_name" "$package_dir"; then
        echo -e "${GREEN}✓ $package_name built successfully${NC}"
    else
        echo -e "${RED}✗ Failed to build $package_name${NC}"
        echo "Build process stopped."
        exit 1
    fi
    
    # For non-metapackage builds, install the package locally to satisfy dependencies
    if [ "$package_name" != "$METAPACKAGE_NAME" ]; then
        search_pattern=$(echo "$package_name" | tr '_' '-')
        deb_file=$(ls "$OUTPUT_DIR"/*${search_pattern}*.deb 2>/dev/null | grep -v dbgsym | head -n 1)
        
        if [ -n "$deb_file" ]; then
            echo -e "${YELLOW}Installing $package_name locally for dependency resolution...${NC}"
            sudo dpkg -i "$deb_file" || sudo apt-get install -f -y
            echo ""
        fi
    fi
done

echo -e "${GREEN}=== Build Complete ===${NC}"
echo ""
echo "Generated packages in $OUTPUT_DIR:"
ls -lh "$OUTPUT_DIR"/ros-${ROS_DISTRO}-*${METAPACKAGE_NAME}*.deb 2>/dev/null | grep -v dbgsym || echo "No packages found"
echo ""
echo -e "${GREEN}To install the metapackage:${NC}"
echo "  sudo dpkg -i $OUTPUT_DIR/ros-${ROS_DISTRO}-${METAPACKAGE_NAME}_*.deb"
echo "  sudo apt-get install -f -y"
