#!/bin/bash
set -euo pipefail

# Stable entry point: clones/updates the repo, then runs the *current* setup/full_setup.sh from it.
# Keep this file minimal and unchanging so a stale downloaded copy still does the right thing.
#
# wget https://raw.githubusercontent.com/DonLakeFlyer/MavlinkTagController2/main/setup/install.sh
# bash install.sh

REPO_URL="https://github.com/DonLakeFlyer/MavlinkTagController2.git"
REPO_DIR="$HOME/repos/MavlinkTagController2"

echo "*** Install git"
sudo apt update
sudo apt install git -y

echo "*** Clone or update MavlinkTagController2"
mkdir -p "$HOME/repos"
if [ ! -d "$REPO_DIR" ]; then
    git clone "$REPO_URL" "$REPO_DIR"
elif [ ! -d "$REPO_DIR/.git" ]; then
    echo "ERROR: $REPO_DIR exists but is not a git checkout; move it aside and re-run." >&2
    exit 1
fi
cd "$REPO_DIR"
# --ff-only: never create a merge commit; fail clearly if local commits/edits block the update.
git pull --ff-only origin main

echo "*** Run setup from the freshly updated repo"
exec bash "$REPO_DIR/setup/full_setup.sh"
