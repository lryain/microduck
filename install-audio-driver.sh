#!/bin/bash
# Standalone script to install TLV320AIC3104 audio codec driver on Radxa Zero 3W
# Extracted from deploy/audio/ and scripts/setup-board.sh

set -e

AUDIO_DIR="$(cd "$(dirname "$0")" && pwd)/deploy/audio"

say() { echo "[AUDIO] $1"; }
warn() { echo "[AUDIO] WARNING: $1" >&2; }

# ── 1. Install required packages ──────────────────────────────────────────────
say "Installing required packages..."
audio_pkgs="alsa-utils device-tree-compiler dkms gcc make i2c-tools"
missing=""
for pkg in $audio_pkgs; do
    dpkg -s "$pkg" >/dev/null 2>&1 || missing="$missing $pkg"
done
if [ -n "$missing" ]; then
    say "Installing:$missing"
    apt-get update -qq || true
    # shellcheck disable=SC2086
    apt-get install -y -qq $missing || { warn "apt failed — audio will not work on this board"; exit 1; }
fi

# ── 2. Install vendor kernel with headers ─────────────────────────────────────
say "Checking vendor kernel..."
if ! dpkg -s linux-image-vendor-rk35xx >/dev/null 2>&1 \
    || ! dpkg -s linux-headers-vendor-rk35xx >/dev/null 2>&1; then
    say "Installing vendor kernel..."
    if ! apt-get install -y -qq \
        linux-image-vendor-rk35xx \
        linux-headers-vendor-rk35xx; then
        warn "could not install the vendor kernel — audio will not work"
        exit 1
    fi
fi

vendor_ver=$(find /lib/modules -maxdepth 1 -name '*-vendor-rk35xx' 2>/dev/null | sort -V | tail -1 | xargs -r basename)
if [ -z "$vendor_ver" ]; then
    warn "no vendor kernel under /lib/modules — audio will not work"
    exit 1
fi

# Make it the active kernel
if [ "$(readlink -f /usr/src/linux)" != "/usr/src/linux-headers-$vendor_ver" ]; then
    say "pointing /usr/src/linux to headers-$vendor_ver"
    ln -sfn "/usr/src/linux-headers-$vendor_ver" /usr/src/linux
fi
if [ "$(readlink -f /lib/modules/$vendor_ver/build)" != "/usr/src/linux-headers-$vendor_ver" ]; then
    ln -sfn "/usr/src/linux-headers-$vendor_ver" "/lib/modules/$vendor_ver/build"
fi

# ── 3. Compile and install device-tree overlays ──────────────────────────────
say "Installing device-tree overlays..."
dtbo_dir="/boot/dtb-${vendor_ver}/rockchip/overlay"
[ -d "$dtbo_dir" ] || dtbo_dir=$(find /boot -maxdepth 3 -type d -path '*/rockchip/overlay' 2>/dev/null | head -1)

if [ -z "$dtbo_dir" ]; then
    warn "no rockchip overlay directory under /boot — audio overlays not installed"
    exit 1
fi

# Helper to add overlay name to armbianEnv.txt
ensure_overlay_word() {
    local name="$1"
    local env_file="/boot/armbianEnv.txt"
    if [ ! -f "$env_file" ]; then
        warn "$env_file not found — cannot add overlay $name"
        return 0
    fi
    if grep -q "^overlays=" "$env_file"; then
        if ! grep "^overlays=" "$env_file" | grep -q "$name"; then
            say "adding $name to armbianEnv.txt overlays"
            sed -i "/^overlays=/ s/$/ $name/" "$env_file"
        fi
    else
        say "adding overlays= line with $name to armbianEnv.txt"
        echo "overlays=$name" >> "$env_file"
    fi
}

for ov_name in i2c3-pihat aic3104-i2c3; do
    ov_dts="$AUDIO_DIR/${ov_name}.dts"
    if [ ! -f "$ov_dts" ]; then
        warn "Source file ${ov_name}.dts not found in $AUDIO_DIR"
        exit 1
    fi
    
    ov_tmp=$(mktemp -d)
    if dtc -@ -I dts -O dtb -o "$ov_tmp/out.dtbo" "$ov_dts" 2>/dev/null; then
        if [ ! -f "$dtbo_dir/rk3568-${ov_name}.dtbo" ] \
            || ! cmp -s "$ov_tmp/out.dtbo" "$dtbo_dir/rk3568-${ov_name}.dtbo"; then
            say "installing ${ov_name}.dtbo"
            cp "$ov_tmp/out.dtbo" "$dtbo_dir/rk3568-${ov_name}.dtbo"
        fi
        ensure_overlay_word "$ov_name"
    else
        warn "could not compile ${ov_name}.dts — audio will not work"
        rm -rf "$ov_tmp"
        exit 1
    fi
    rm -rf "$ov_tmp"
done

# ── 4. Install codec driver via DKMS ─────────────────────────────────────────
say "Installing codec driver via DKMS..."
dkms_src_dir="$AUDIO_DIR/aic3x-dkms"
if [ ! -f "$dkms_src_dir/dkms.conf" ]; then
    warn "DKMS source not found in $dkms_src_dir"
    exit 1
fi

dkms_ver=$(sed -n 's/^PACKAGE_VERSION="\(.*\)"$/\1/p' "$dkms_src_dir/dkms.conf")
dkms_dst="/usr/src/aic3x-$dkms_ver"

# Deploy sources if needed
deploy_needed=0
for f in dkms.conf Makefile tlv320aic3x.c tlv320aic3x.h tlv320aic3x-i2c.c; do
    if [ ! -f "$dkms_dst/$f" ] || ! cmp -s "$dkms_src_dir/$f" "$dkms_dst/$f"; then
        deploy_needed=1
        break
    fi
done

if [ "$deploy_needed" = 1 ]; then
    say "deploying aic3x DKMS sources to $dkms_dst"
    dkms remove "aic3x/$dkms_ver" --all >/dev/null 2>&1 || true
    mkdir -p "$dkms_dst"
    cp "$dkms_src_dir"/* "$dkms_dst/"
fi

if dkms status "aic3x/$dkms_ver" 2>/dev/null | grep "$vendor_ver" | grep -q installed; then
    say "aic3x DKMS module already installed"
else
    # Rebuild vendor headers' host tools if needed
    if [ -d "/usr/src/linux-headers-$vendor_ver" ] \
        && [ ! -x "/usr/src/linux-headers-$vendor_ver/scripts/mod/modpost" ]; then
        say "rebuilding the vendor headers' host tools (modpost)"
        dpkg-reconfigure linux-headers-vendor-rk35xx >/dev/null 2>&1 || true
    fi
    
    say "building the aic3x codec driver via DKMS (takes a minute)"
    if dkms install "aic3x/$dkms_ver" -k "$vendor_ver"; then
        say "aic3x DKMS module installed for $vendor_ver"
    else
        warn "DKMS build failed — audio will not work"
        warn "see /var/lib/dkms/aic3x/$dkms_ver/build/make.log"
        exit 1
    fi
fi

# ── 5. Install mixer init script and service ─────────────────────────────────
say "Installing mixer init script and service..."
init_script="$AUDIO_DIR/aic3104-init.sh"
if [ ! -f "$init_script" ]; then
    warn "aic3104-init.sh not found in $AUDIO_DIR"
    exit 1
fi

if [ ! -f /usr/local/bin/aic3104-init.sh ] \
    || ! cmp -s "$init_script" /usr/local/bin/aic3104-init.sh; then
    say "installing /usr/local/bin/aic3104-init.sh"
    install -m 755 "$init_script" /usr/local/bin/aic3104-init.sh
fi

# Create systemd service
cat > /tmp/aic3104-init.service <<'UNIT'
[Unit]
Description=TLV320AIC3104 mixer init
After=systemd-modules-load.service
# No ConditionPathExists: the sound card probe is deferred until the DKMS codec module
# autoloads, so the card can appear seconds into boot — the script polls for it instead.
Before=robotd.service

[Service]
Type=oneshot
ExecStart=/usr/local/bin/aic3104-init.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT

if [ ! -f /etc/systemd/system/aic3104-init.service ] \
    || ! cmp -s /tmp/aic3104-init.service /etc/systemd/system/aic3104-init.service; then
    say "installing aic3104-init.service"
    install -m 644 /tmp/aic3104-init.service /etc/systemd/system/aic3104-init.service
    systemctl daemon-reload
fi
systemctl is-enabled --quiet aic3104-init.service \
    || systemctl enable aic3104-init.service >/dev/null 2>&1 || true
rm -f /tmp/aic3104-init.service

say "Audio driver installation complete!"
say "Please reboot the system for changes to take effect."