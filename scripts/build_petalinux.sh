#!/bin/bash
set -e

# === Configuration ===
PL_DIR=./pl
PETALINUX_DIR=./Petalinux
PROJECT_NAME=klab_project
VIVADO_VER=2024.2

REPO_ROOT=$(pwd)  # Save the original repo directory
SYSTEM_DTSI=$REPO_ROOT/src-petalinux/system-user.dtsi

XSA_NAME=${PROJECT_NAME}.xsa
XSA_PATH=$REPO_ROOT/vivado_project/$XSA_NAME

# === Step 0: Build Vivado project ===
echo "=== [0/7] Building Vivado project ==="
echo "Commented out!"
# vivado -mode batch -source $PL_DIR/scripts/create_project.tcl
# vivado -mode batch -source $PL_DIR/scripts/build_bitstream.tcl

# === Step 1: Create PetaLinux project if missing ===
if [ ! -f "$PETALINUX_DIR/project-spec/meta-user/conf/petalinuxbsp.conf" ]; then
  echo "=== [0] Creating new PetaLinux project ==="
  petalinux-create --type project --template zynq --name $(basename $PETALINUX_DIR)
fi

# === Step 2: Configure PetaLinux project ===
cd $PETALINUX_DIR
echo "=== [2/7] Configuring PetaLinux with XSA ==="

if [ ! -f "$XSA_PATH" ]; then
  echo "ERROR: XSA not found at $XSA_PATH"
  exit 1
fi

petalinux-config --get-hw-description=$XSA_PATH --silentconfig

# === Step 3: Add system-user.dtsi ===
echo "=== [3/7] Adding custom system-user.dtsi ==="
DTSI_DEST=project-spec/meta-user/recipes-bsp/device-tree/files/
mkdir -p $DTSI_DEST
cp $SYSTEM_DTSI $DTSI_DEST/system-user.dtsi

cat <<EOF > project-spec/meta-user/recipes-bsp/device-tree/device-tree.bbappend
FILESEXTRAPATHS:prepend := "\${THISDIR}/files:"
SRC_URI += "file://system-user.dtsi"
EOF

# === Step 4: Kernel Config for UIO ===
echo "=== [4/7] Adding kernel config for UIO ==="
KERNEL_CFG_DIR=project-spec/meta-user/recipes-kernel/linux/linux-xlnx
mkdir -p $KERNEL_CFG_DIR

cat <<EOF > $KERNEL_CFG_DIR/uio.cfg
CONFIG_UIO=y
CONFIG_UIO_PDRV_GENIRQ=y
EOF

cat <<EOF > $KERNEL_CFG_DIR/linux-xlnx_%.bbappend
FILESEXTRAPATHS:prepend := "\${THISDIR}/linux-xlnx:"
SRC_URI += "file://uio.cfg"
EOF

# === Step 5: RootFS Config ===
echo "=== [5/7] Setting rootfs options ==="
ROOTFS_CFG=project-spec/configs/rootfs_config

for opt in \
  "CONFIG_packagegroup-core-buildessential=y" \
  "CONFIG_imagefeature-empty-root-password=y" \
  "CONFIG_imagefeature-serial-autologin-root=y"
do
  grep -q "$opt" $ROOTFS_CFG || echo "$opt" >> $ROOTFS_CFG
done

# Also reinforce with bbappend
IMAGE_BBAPPEND=project-spec/meta-user/recipes-core/images/petalinux-image.bbappend
mkdir -p $(dirname $IMAGE_BBAPPEND)
cat <<EOF > $IMAGE_BBAPPEND
IMAGE_FEATURES:append = " empty-root-password serial-autologin-root"
IMAGE_INSTALL:append = " packagegroup-core-buildessential"
EOF

# === Step 6: Build PetaLinux ===
echo "=== [6/7] Building PetaLinux image ==="
petalinux-build

# === Step 7: Package BOOT.BIN for SD boot ===
echo "=== [7/7] Packaging BOOT.BIN for SD card boot ==="
BOOT_OUT=images/linux

petalinux-package --boot \
  --fsbl $BOOT_OUT/zynq_fsbl.elf \
  --fpga $BOOT_OUT/system.bit \
  --u-boot \
  --force

echo "=== ✅ SD card image files are in: $BOOT_OUT ==="
echo "    Copy the following to the FAT32 partition of your SD card:"
echo "      - BOOT.BIN"
echo "      - image.ub"

echo "=== ✅ DONE: PetaLinux build complete ==="
