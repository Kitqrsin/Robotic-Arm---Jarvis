#!/usr/bin/env bash
# cleanup.sh — Remove junk files and prune Docker cache

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Colour

separator() { echo -e "${CYAN}────────────────────────────────────────────────────────────${NC}"; }

echo -e "${CYAN}"
echo "  ██████╗██╗     ███████╗ █████╗ ███╗   ██╗██╗   ██╗██████╗ "
echo " ██╔════╝██║     ██╔════╝██╔══██╗████╗  ██║██║   ██║██╔══██╗"
echo " ██║     ██║     █████╗  ███████║██╔██╗ ██║██║   ██║██████╔╝"
echo " ██║     ██║     ██╔══╝  ██╔══██║██║╚██╗██║██║   ██║██╔═══╝ "
echo " ╚██████╗███████╗███████╗██║  ██║██║ ╚████║╚██████╔╝██║     "
echo "  ╚═════╝╚══════╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝ ╚═════╝ ╚═╝     "
echo -e "${NC}"
echo -e "  Robotic-Arm Jarvis — Project Cleanup Script"
separator

# ─────────────────────────────────────────────────────────────────────────────
# 1. Core dump files (core.<pid>)
# ─────────────────────────────────────────────────────────────────────────────
separator
echo -e "${YELLOW}[1/5] Removing core dump files...${NC}"

CORE_FILES=("$SCRIPT_DIR"/core.*)
if compgen -G "$SCRIPT_DIR/core.*" > /dev/null 2>&1; then
    for f in "${CORE_FILES[@]}"; do
        echo -e "  ${RED}✗${NC} Deleting: $(basename "$f")"
        rm -f "$f"
    done
    echo -e "${GREEN}  ✔ Core dump files removed.${NC}"
else
    echo -e "${GREEN}  ✔ No core dump files found.${NC}"
fi

# ─────────────────────────────────────────────────────────────────────────────
# 2. Python cache (__pycache__ and .pyc files)
# ─────────────────────────────────────────────────────────────────────────────
separator
echo -e "${YELLOW}[2/5] Removing Python cache files...${NC}"

find "$SCRIPT_DIR" -type d -name "__pycache__" -print -exec rm -rf {} + 2>/dev/null || true
find "$SCRIPT_DIR" -type f -name "*.pyc" -print -delete 2>/dev/null || true
find "$SCRIPT_DIR" -type f -name "*.pyo" -print -delete 2>/dev/null || true
echo -e "${GREEN}  ✔ Python cache cleaned.${NC}"

# ─────────────────────────────────────────────────────────────────────────────
# 3. Colcon / ROS 2 build logs
# ─────────────────────────────────────────────────────────────────────────────
separator
echo -e "${YELLOW}[3/5] Removing old Colcon build logs...${NC}"

LOG_DIR="$SCRIPT_DIR/log"
if [ -d "$LOG_DIR" ]; then
    # Keep the COLCON_IGNORE sentinel; remove all dated build_* sub-dirs
    REMOVED=0
    while IFS= read -r -d '' d; do
        echo -e "  ${RED}✗${NC} Removing log dir: $(basename "$d")"
        # Use sudo if we lack write permission (files created by root via Docker/sudo)
        if rm -rf "$d" 2>/dev/null; then
            : # removed without sudo
        else
            echo -e "  ${YELLOW}  → retrying with sudo...${NC}"
            sudo rm -rf "$d"
        fi
        REMOVED=$((REMOVED + 1))
    done < <(find "$LOG_DIR" -mindepth 1 -maxdepth 1 -type d -name "build_*" -print0)

    # Remove dangling symlinks (latest, latest_build)
    find "$LOG_DIR" -maxdepth 1 -type l -delete 2>/dev/null || sudo find "$LOG_DIR" -maxdepth 1 -type l -delete 2>/dev/null || true

    if [ "$REMOVED" -eq 0 ]; then
        echo -e "${GREEN}  ✔ No old build logs found.${NC}"
    else
        echo -e "${GREEN}  ✔ Removed $REMOVED log director(ies).${NC}"
    fi
else
    echo -e "${GREEN}  ✔ No log directory found.${NC}"
fi

# ─────────────────────────────────────────────────────────────────────────────
# 4. Miscellaneous hidden junk files
# ─────────────────────────────────────────────────────────────────────────────
separator
echo -e "${YELLOW}[4/5] Removing miscellaneous junk files...${NC}"

# Temporary / editor / OS junk patterns
JUNK_PATTERNS=("*.tmp" "*.swp" "*.swo" "*~" ".DS_Store" "Thumbs.db" ".lgd-*")

for pattern in "${JUNK_PATTERNS[@]}"; do
    while IFS= read -r -d '' f; do
        echo -e "  ${RED}✗${NC} Removing: $f"
        rm -f "$f"
    done < <(find "$SCRIPT_DIR" -not -path '*/.git/*' -type f -name "$pattern" -print0 2>/dev/null)
done
echo -e "${GREEN}  ✔ Miscellaneous junk removed.${NC}"

# ─────────────────────────────────────────────────────────────────────────────
# 5. Docker system prune
# ─────────────────────────────────────────────────────────────────────────────
separator
echo -e "${YELLOW}[5/5] Cleaning Docker cache...${NC}"

if command -v docker &> /dev/null; then
    echo -e "  Removing stopped containers, dangling images, unused networks & build cache..."
    docker system prune -f
    echo ""
    echo -e "  ${YELLOW}(Optional)${NC} To also remove ALL unused images (not just dangling), run:"
    echo -e "      docker system prune -af"
    echo -e "${GREEN}  ✔ Docker cache cleaned.${NC}"
else
    echo -e "${YELLOW}  ⚠ Docker not found — skipping.${NC}"
fi

# ─────────────────────────────────────────────────────────────────────────────
separator
echo -e "${GREEN}  All done! Project cleaned successfully.${NC}"
separator
