#!/usr/bin/env bash
# PC <-> STM32 lewat terminal (shell script)
# Frame 8 byte: [FF FE ID CMD 00 00 00 CHK]
# CHK = XOR byte 0..6
# STM mengirim hello (CMD 0xFF) saat boot/reset -> PC membalas CMD 1 (LED ON)
# Pakai: ./ledcontrol.sh /dev/stm32 [baud]

PORT="${1:-/dev/stm32}"
BAUD="${2:-115200}"
H1=255; H2=254; STM_ID=$((0x67)); FRAME_LEN=8; CMD_HELLO=255

# tunggu port muncul (penting saat boot)
echo "Menunggu port $PORT ..."
while [[ ! -e $PORT ]]; do sleep 1; done

stty -F "$PORT" "$BAUD" raw -echo -echoe -echok cs8 -cstopb -parenb -hupcl min 1 time 0 \
  || { echo "Gagal konfigurasi $PORT"; exit 1; }
exec 3<>"$PORT"

hexs() { local s=""; for b in "$@"; do s+=$(printf "%02X " "$b"); done; echo "${s% }"; }

send_cmd() {
  local cmd=$1
  local -a f=($H1 $H2 $STM_ID "$cmd" 0 0 0 0)
  local chk=0 i out=""
  for ((i = 0; i < FRAME_LEN - 1; i++)); do chk=$(( chk ^ f[i] )); done
  f[7]=$chk
  for b in "${f[@]}"; do out+=$(printf '\\x%02x' "$b"); done
  printf "$out" >&3
  echo "[TX] $(hexs "${f[@]}")"
}

handle_frame() {
  local -a f=("$@")
  if (( f[2] == STM_ID )); then
    echo "[RX] $(hexs "${f[@]}")"
    if (( f[3] == CMD_HELLO )); then
      echo "[STM terdeteksi/reset] ID=$(printf '0x%02X' "${f[2]}") -> kirim LED ON"
      send_cmd 1
    fi
  else
    printf "[RX ID asing 0x%02X] %s\n" "${f[2]}" "$(hexs "${f[@]}")"
  fi
}

rx_loop() {
  local -a buf=()
  local b chk i
  while true; do
    b=$(dd bs=1 count=1 <&3 2>/dev/null | od -An -v -tu1)
    b=${b//[[:space:]]/}
    [[ -z $b ]] && { sleep 0.01; continue; }
    buf+=("$b")

    while (( ${#buf[@]} >= FRAME_LEN )); do
      # cari header
      if (( buf[0] != H1 || buf[1] != H2 )); then
        buf=("${buf[@]:1}"); continue
      fi
      # cek checksum (XOR byte 0..6)
      chk=0
      for ((i = 0; i < FRAME_LEN - 1; i++)); do chk=$(( chk ^ buf[i] )); done
      if (( chk != buf[7] )); then
        buf=("${buf[@]:1}"); continue
      fi
      handle_frame "${buf[@]:0:FRAME_LEN}"
      buf=("${buf[@]:FRAME_LEN}")
    done
  done
}

cleanup() {
  trap - INT TERM EXIT
  [[ -n $RX_PID ]] && { pkill -P "$RX_PID" 2>/dev/null; kill "$RX_PID" 2>/dev/null; }
  exec 3>&-
  exit 0
}
trap cleanup INT TERM EXIT

rx_loop &
RX_PID=$!

# kalau STM sudah menyala duluan, langsung kirim LED ON
echo "Menunggu STM dengan ID 0x67 ..."
sleep 0.3
send_cmd 1

if [[ -t 0 ]]; then
  # mode manual (dijalankan dari terminal)
  echo "Ketik 1 = LED ON, 0 = LED OFF, q = keluar"
  while read -rp "> " s; do
    case "$s" in
      1) send_cmd 1 ;;
      0) send_cmd 0 ;;
      q) break ;;
      *) echo "Input hanya 0 atau 1" ;;
    esac
  done
else
  # mode service (tanpa terminal): tetap jalan di latar belakang
  wait "$RX_PID"
fi