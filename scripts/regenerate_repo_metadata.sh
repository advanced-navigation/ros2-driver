#!/bin/bash
set -e

# Configuration
REMOTE_SERVER="avocado.acfr.usyd.edu.au"
REMOTE_USER="jjustin"
REMOTE_REPO_DIR="/data/www/EHM/datasets/ubuntu-repo"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Repository Metadata Regeneration Script ===${NC}"
echo "Remote Server: $REMOTE_SERVER"
echo "Remote Repository: $REMOTE_REPO_DIR"
echo ""

# Function to check if SSH connection works
check_ssh() {
    echo -e "${YELLOW}Checking SSH connection to $REMOTE_SERVER...${NC}"
    if ssh -o ConnectTimeout=5 "$REMOTE_USER@$REMOTE_SERVER" "echo 'SSH connection successful'" >/dev/null 2>&1; then
        echo -e "${GREEN}✓ SSH connection successful${NC}"
        return 0
    else
        echo -e "${RED}✗ SSH connection failed${NC}"
        echo "Please ensure:"
        echo "  1. SSH key is set up for $REMOTE_USER@$REMOTE_SERVER"
        echo "  2. You have access to the server"
        echo "  3. Server is reachable from your network"
        return 1
    fi
}

# Function to regenerate repository metadata
regenerate_metadata() {
    echo -e "${YELLOW}Regenerating repository metadata on $REMOTE_SERVER...${NC}"
    echo ""
    
    # Create a script to run on the remote server (RHEL compatible)
    local remote_script=$(cat << 'EOF'
#!/bin/bash
set -e

REPO_DIR="/data/www/EHM/datasets/ubuntu-repo"
cd "$REPO_DIR"

echo "Current directory: $(pwd)"
echo "Listing .deb files in pool/main:"
ls -lh pool/main/*.deb 2>/dev/null | tail -n 10 || echo "No .deb files found"
echo ""

echo "Generating Packages file..."
dpkg-scanpackages -m pool/main /dev/null > dists/noble/main/binary-amd64/Packages

echo "Compressing Packages file..."
gzip -9c dists/noble/main/binary-amd64/Packages > dists/noble/main/binary-amd64/Packages.gz

echo "Generating Release file..."
cd dists/noble
cat > Release << EOL
Origin: ACFR
Label: ACFR Ubuntu Repository
Suite: noble
Codename: noble
Architectures: amd64
Components: main
Description: ACFR Ubuntu Repository for ROS 2 Jazzy
Date: $(LANG=C date -u '+%a, %d %b %Y %H:%M:%S +0000')
EOL

# Add file hashes to Release (RHEL uses --format instead of -c)
echo "MD5Sum:" >> Release
for file in main/binary-amd64/Packages main/binary-amd64/Packages.gz; do
    if [ -f "$file" ]; then
        echo " $(md5sum $file | cut -d' ' -f1) $(stat --format=%s $file) $file" >> Release
    fi
done

echo "SHA1:" >> Release
for file in main/binary-amd64/Packages main/binary-amd64/Packages.gz; do
    if [ -f "$file" ]; then
        echo " $(sha1sum $file | cut -d' ' -f1) $(stat --format=%s $file) $file" >> Release
    fi
done

echo "SHA256:" >> Release
for file in main/binary-amd64/Packages main/binary-amd64/Packages.gz; do
    if [ -f "$file" ]; then
        echo " $(sha256sum $file | cut -d' ' -f1) $(stat --format=%s $file) $file" >> Release
    fi
done

echo ""
echo "Generated files:"
ls -lh main/binary-amd64/Packages* Release 2>/dev/null
echo ""

echo "Package count in repository:"
PACKAGE_COUNT=$(grep -c "^Package:" main/binary-amd64/Packages || echo "0")
echo "$PACKAGE_COUNT packages"
echo ""

echo "Sample packages (first 20):"
grep "^Package:" main/binary-amd64/Packages | head -n 20
echo ""

if [ $PACKAGE_COUNT -gt 20 ]; then
    echo "... and $((PACKAGE_COUNT - 20)) more packages"
    echo ""
fi

echo "Repository metadata regenerated successfully!"
EOF
)
    
    # Execute the script on the remote server
    if ssh "$REMOTE_USER@$REMOTE_SERVER" "bash -s" <<< "$remote_script"; then
        echo -e "${GREEN}✓ Repository metadata regenerated successfully${NC}"
        return 0
    else
        echo -e "${RED}✗ Failed to regenerate repository metadata${NC}"
        return 1
    fi
}

# Main process
main() {
    # Check SSH connection
    if ! check_ssh; then
        exit 1
    fi
    echo ""
    
    # Regenerate metadata
    if ! regenerate_metadata; then
        exit 1
    fi
    
    echo ""
    echo -e "${GREEN}=== Metadata Regeneration Complete ===${NC}"
    echo ""
    echo "The repository metadata has been updated."
    echo "Clients can now run 'apt update' to see the latest packages."
    echo ""
}

# Run main function
main
