#!/usr/bin/env bash
set -e
[[ $EUID -eq 0 ]] || { echo "Jalankan: sudo $0"; exit 1; }

SRC="$(dirname "$(readlink -f "$0")")/ledcontrol.sh"
install -m 755 "$SRC" /usr/local/bin/ledcontrol.sh

# symlink /dev/stm32 setiap FTDI dicolok
cat > /etc/udev/rules.d/99-stm32.rules <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="stm32", TAG+="systemd", MODE="0666"
EOF

# service: start saat /dev/stm32 muncul, stop saat dicabut
cat > /etc/systemd/system/ledcontrol.service <<'EOF'
[Unit]
Description=PC <-> STM32 LED control
BindsTo=dev-stm32.device
After=dev-stm32.device

[Service]
ExecStart=/usr/local/bin/ledcontrol.sh /dev/stm32 115200
Restart=on-failure
RestartSec=2

[Install]
WantedBy=dev-stm32.device
EOF

udevadm control --reload
systemctl daemon-reload
systemctl enable ledcontrol.service
udevadm trigger --subsystem-match=tty --action=add
echo "Selesai. Cek: journalctl -u ledcontrol -f"