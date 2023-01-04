#!/usr/bin/env bash
set -o errexit -o nounset -o pipefail
IFS=$'\n\t\v'
cd `dirname "${BASH_SOURCE[0]:-$0}"`/..

# hard dependencies we'll always need
# there's more further down the script, only checked if actually needed
# can't check for: libudev-dev
BIN_DEPENDENCIES=(gcc pkg-config)
LIB_DEPENDENCIES=(libssl)

function checkdep() {
  if command -v $1 >/dev/null; then
    return 0
  else
    return 1
  fi
}
function ensuredep() {
  if ! checkdep $1; then
    >&2 echo "Binary '$1' not found. Please install it using your systems package manager"
    exit 1
  fi
}

function checklib() {
  if ldconfig -p | grep $1 >/dev/null; then
    return 0
  else
    return 1
  fi
}
function ensurelib() {
  if ! checklib $1; then
    >&2 echo "Library '$1' not found. Please install it using your systems package manager"
    exit 1
  fi
}

for b in "${BIN_DEPENDENCIES[@]}"; do
  ensuredep $b
done

for l in "${LIB_DEPENDENCIES[@]}"; do
  ensurelib $l
done

if ! checkdep rustup; then
  >&2 echo "Installing rustup..."
  ensuredep curl
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain stable -y
fi

if ! checkdep cargo-flash; then
  >&2 echo "Installing cargo-flash..."
  cargo install cargo-flash
fi

if ! checkdep flip-link; then
  >&2 echo "Installing flip-link..."
  cargo install flip-link
fi

# if ! rustup show active-toolchain | grep nightly | grep default >/dev/null; then
#   >&2 echo "Setting up nightly cargo toolchain..."
#   rustup default nightly
# fi

TARGET_NAME='thumbv7em-none-eabihf'
if ! rustup target list | grep ${TARGET_NAME} | grep installed >/dev/null; then
  >&2 echo "Installing target ${TARGET_NAME}..."
  rustup target add ${TARGET_NAME}
fi

# udev rules need to be present to be able to flash as non-root
if [[ $(id -u) -ne 0 ]]; then
  if [[ ! -f /etc/udev/rules.d/49-stlinkv1.rules ]]; then
    ensuredep sudo
    sudo cp udev_rules/* /etc/udev/rules.d
    sudo udevadm control --reload
    sudo udevadm trigger
  fi
fi

>&2 echo "Toolchain is OK"