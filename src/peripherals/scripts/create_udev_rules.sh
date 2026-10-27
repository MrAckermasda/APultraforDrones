#!/usr/bin/env bash
set -euo pipefail
rule_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for rule in "${rule_dir}"/*.rules; do
    sudo install -m 0644 "$rule" "/etc/udev/rules.d/$(basename -- "$rule")"
done
sudo udevadm control --reload-rules
printf '%s\n' '规则已重新加载；重新插拔对应设备后生效。'
