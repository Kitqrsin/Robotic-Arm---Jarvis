#!/bin/bash
# Camera Stream Helper for Docker
# Runs on the HOST to capture from Pi Camera via rpicam-still (timelapse mode).
# Writes JPEG frames to a shared workspace directory that the Docker TKinter GUI reads.
#
# Usage: ./camera_stream.sh [start|stop|status]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRAME_FILE="${SCRIPT_DIR}/.camera_frame.jpg"
TMP_DIR="${SCRIPT_DIR}/.cam_frames"
PID_FILE="/tmp/jarvis_camera_stream.pid"

start_stream() {
    # Kill any existing stream
    stop_stream 2>/dev/null

    mkdir -p "$TMP_DIR"

    # Use rpicam-still in timelapse mode:
    #   --mode 1640:1232   → force full-sensor 2×2 binned mode (max FOV)
    #   --roi 0,0,1,1      → no digital zoom – use entire sensor area
    #   --timelapse 66ms   → capture every 66 ms (~15 FPS)
    #   --nopreview        → headless
    #   -t 0               → run forever
    #   --width/height     → output resolution (scaled from sensor mode)
    #   -q 50              → JPEG quality (lower = smaller + faster)
    #
    # NOTE: We write frames into a tmp dir and then copy the latest to
    # FRAME_FILE as a regular file (not symlink) so Docker volume mounts
    # can resolve it.  rpicam-still --latest creates an absolute symlink
    # that won't work inside Docker.
    rpicam-still \
        --width 640 --height 480 \
        --mode 1640:1232 \
        --roi 0,0,1,1 \
        --timelapse 33 \
        --nopreview \
        -t 0 \
        -q 50 \
        --framestart 0 \
        -o "${TMP_DIR}/frame_%06d.jpg" \
        2>/dev/null &

    RPICAM_PID=$!

    # Background loop: copy the latest frame to FRAME_FILE as a regular file
    (
        while kill -0 "$RPICAM_PID" 2>/dev/null; do
            LATEST=$(ls -1t "${TMP_DIR}"/frame_*.jpg 2>/dev/null | head -1)
            if [ -n "$LATEST" ]; then
                cp -f "$LATEST" "${FRAME_FILE}.tmp" 2>/dev/null && \
                    mv -f "${FRAME_FILE}.tmp" "$FRAME_FILE" 2>/dev/null
            fi
            sleep 0.03
        done
    ) &

    echo "$RPICAM_PID" > "$PID_FILE"
    sleep 1

    if kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        echo "[camera_stream] Started (PID=$(cat $PID_FILE))"
        echo "[camera_stream] Latest frame: $FRAME_FILE"
        echo "[camera_stream]   (inside Docker: /workspace/.camera_frame.jpg)"

        # Background cleanup: periodically remove old frame files to save disk
        (
            while kill -0 "$(cat "$PID_FILE" 2>/dev/null)" 2>/dev/null; do
                sleep 10
                # Keep only the 5 newest frame files, delete older ones
                ls -1t "${TMP_DIR}"/frame_*.jpg 2>/dev/null | tail -n +6 | xargs rm -f 2>/dev/null
            done
        ) &
    else
        echo "[camera_stream] ERROR: rpicam-still failed to start"
        cat "$PID_FILE" 2>/dev/null
        rm -f "$PID_FILE"
        return 1
    fi
}

stop_stream() {
    if [ -f "$PID_FILE" ]; then
        kill "$(cat "$PID_FILE")" 2>/dev/null
        wait "$(cat "$PID_FILE")" 2>/dev/null
        rm -f "$PID_FILE"
    fi
    pkill -f "rpicam-still.*jarvis_cam" 2>/dev/null
    rm -f "$FRAME_FILE"
    rm -rf "$TMP_DIR"
    echo "[camera_stream] Stopped"
}

status_stream() {
    if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
        echo "[camera_stream] Running (PID=$(cat $PID_FILE))"
        if [ -f "$FRAME_FILE" ]; then
            echo "[camera_stream] Latest frame: $(ls -la "$FRAME_FILE" 2>/dev/null)"
        else
            echo "[camera_stream] Waiting for first frame..."
        fi
    else
        echo "[camera_stream] Not running"
    fi
}

case "${1:-start}" in
    start)  start_stream ;;
    stop)   stop_stream ;;
    status) status_stream ;;
    *)      echo "Usage: $0 [start|stop|status]" ;;
esac
