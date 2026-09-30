#!/usr/bin/env bash
# 4: V-blocks clamp the flask, liquid doser fills to the ring mark while the dtv camera is recorded,
#    V-blocks open again. Video: recordings/dtv_<timestamp>.mp4 (frames from /dtv/image, 2 fps).
#    RING_ROW=396 ./04_dilute_to_volume.sh  pins the ring row (camera at Z=16.7 mm); OPEN_MM sets the V-block opening.
source "$(dirname "$0")/lib.sh"
D="$(dirname "$(readlink -f "$0")")"; mkdir -p "$D/recordings"
VID="$D/recordings/dtv_$(date +%Y%m%d_%H%M%S).mp4"
step dtv v clamp
( cd "$D/../.." && source rosenv.sh && exec python3 "$D/record_dtv.py" "$VID" ) & REC=$!
trap 'kill $REC 2>/dev/null; wait $REC 2>/dev/null' EXIT
sleep 1
step run dilute-to-volume --flask-ml "${FLASK_ML:-25}" ${RING_ROW:+--ring-row "$RING_ROW"}
kill -INT $REC; wait $REC || true; trap - EXIT
echo "video: $VID"
step dtv v open "${OPEN_MM:-10}"
