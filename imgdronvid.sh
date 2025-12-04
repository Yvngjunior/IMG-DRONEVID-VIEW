#!/usr/bin/env bash
set -euo pipefail

# SOFTWARE-DRONE: Smooth glide automatic detector + zoom (medium)
# Usage: ./drone_zoom.sh input.jpg [output.mp4]
#
# Defaults: grid=10x10, top_k=5, frames_per_segment=60 (30 FPS -> 2s per segment)
# Medium zoom factor ~ 1.4 (i.e., crop viewport is img/1.4 so it looks moderately zoomed)

IMG="$1"
OUT="${2:-drone_output.mp4}"

# Config (tweak if you want)
GRID=10               # GRID x GRID cells
TOP_K=5               # how many top-detail cells to visit
FRAMES_PER_SEG=60     # frames interpolated between waypoints (smoothness)
FPS=30
ZOOM_MEDIUM=1.4       # medium zoom factor
TMPDIR="$(mktemp -d)"
USE_MAGICK=""         # will be set to "magick" or "convert" based on availability

cleanup() { rm -rf "$TMPDIR"; }
trap cleanup EXIT

# find which ImageMagick binary exists
if command -v magick >/dev/null 2>&1; then
  USE_MAGICK="magick"
elif command -v convert >/dev/null 2>&1; then
  USE_MAGICK="convert"
else
  echo "Error: ImageMagick not found. Install it: pkg install imagemagick" >&2
  exit 1
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "Error: ffmpeg not found. Install it: pkg install ffmpeg" >&2
  exit 1
fi

# sanitize input path
if [ ! -f "$IMG" ]; then
  echo "Error: input file not found: $IMG" >&2
  exit 1
fi

echo "Starting analysis on: $IMG"
echo "Temporary dir: $TMPDIR"

# Get image dimensions
# identify -format "%w %h" filename
read IMG_W IMG_H < <($USE_MAGICK identify -format "%w %h\n" "$IMG")
echo "Image size: ${IMG_W}x${IMG_H}"

# Step 1: make an edge map (grayscale + Canny)
EDGES="$TMPDIR/edges.png"
# Canny parameters tuned for general images — they can be changed if you want more/fewer edges
$USE_MAGICK "$IMG" -colorspace Gray -canny 0x1+10%+30% "$EDGES"
echo "Edge map generated."

# Step 2: split edges into GRID x GRID and compute mean intensity (detail score per cell)
cell_w=$(( IMG_W / GRID ))
cell_h=$(( IMG_H / GRID ))

# arrays for centers and scores
declare -a centers_x
declare -a centers_y
declare -a scores

# compute cell scores
index=0
for gy in $(seq 0 $((GRID-1))); do
  for gx in $(seq 0 $((GRID-1))); do
    # cell top-left
    xoff=$(( gx * cell_w ))
    yoff=$(( gy * cell_h ))

    # last column/row may need to include remaining pixels
    cur_w=$cell_w
    cur_h=$cell_h
    if [ "$gx" -eq $((GRID-1)) ]; then cur_w=$(( IMG_W - xoff )); fi
    if [ "$gy" -eq $((GRID-1)) ]; then cur_h=$(( IMG_H - yoff )); fi

    # crop the cell from edges image and get mean intensity (0..1)
    crop_file="$TMPDIR/cell_${gx}_${gy}.png"
    $USE_MAGICK "$EDGES" -crop "${cur_w}x${cur_h}+${xoff}+${yoff}" +repage "$crop_file"

    # use identify to get mean (ImageMagick info: %[mean] returns 0..QuantumRange; but identify -format "%[mean]" outputs a value 0..1)
    mean=$( $USE_MAGICK identify -format "%[mean]\n" "$crop_file" )
    # store center coords and mean
    cx=$(( xoff + cur_w/2 ))
    cy=$(( yoff + cur_h/2 ))
    centers_x[$index]=$cx
    centers_y[$index]=$cy
    scores[$index]=$mean
    index=$(( index+1 ))
  done
done

# Step 3: select top-K cells by score
# We'll create a temp file with "index score cx cy" lines and sort it
score_file="$TMPDIR/scores.tsv"
: > "$score_file"
for i in "${!scores[@]}"; do
  printf "%d\t%.8f\t%d\t%d\n" "$i" "${scores[$i]}" "${centers_x[$i]}" "${centers_y[$i]}" >> "$score_file"
done

# sort descending by score, pick top K
topfile="$TMPDIR/topk.tsv"
sort -k2,2nr "$score_file" | head -n "$TOP_K" > "$topfile"

# waypoints: start at center, then topK centers (in sorted order), then back to center
start_cx=$(( IMG_W / 2 ))
start_cy=$(( IMG_H / 2 ))
declare -a wp_x
declare -a wp_y
declare -a wp_zoom

# push start
wp_x+=("$start_cx")
wp_y+=("$start_cy")
wp_zoom+=(1.0)   # start with no zoom

# read topk and append centers (we will visit them in descending detail order)
while IFS=$'\t' read -r idx sc cx cy; do
  wp_x+=("$cx")
  wp_y+=("$cy")
  # give each waypoint a medium zoom factor (you might vary per score if desired)
  wp_zoom+=("$ZOOM_MEDIUM")
done < "$topfile"

# push final: return to center (zoom=1)
wp_x+=("$start_cx")
wp_y+=("$start_cy")
wp_zoom+=(1.0)

NUM_WP=${#wp_x[@]}
echo "Waypoints: $NUM_WP (start + top${TOP_K} + return)"
for i in $(seq 0 $((NUM_WP-1))); do
  echo "  WP$i -> x=${wp_x[$i]} y=${wp_y[$i]} zoom=${wp_zoom[$i]}"
done

# Step 4: generate frames by interpolating between waypoints
FRAME_INDEX=0
echo "Generating frames..."
for seg in $(seq 0 $((NUM_WP-2))); do
  # from waypoint seg to seg+1
  xA=${wp_x[$seg]}; yA=${wp_y[$seg]}; zA=${wp_zoom[$seg]}
  xB=${wp_x[$((seg+1))]}; yB=${wp_y[$((seg+1))]}; zB=${wp_zoom[$((seg+1))]}

  for f in $(seq 0 $((FRAMES_PER_SEG-1))); do
    t=$(awk -v f="$f" -v N="$((FRAMES_PER_SEG-1))" 'BEGIN{ if(N==0){print 0;} else print f/N }')
    # linear interpolation (LERP)
    cx=$(awk -v a="$xA" -v b="$xB" -v t="$t" 'BEGIN{printf("%d", a + (b-a)*t)}')
    cy=$(awk -v a="$yA" -v b="$yB" -v t="$t" 'BEGIN{printf("%d", a + (b-a)*t)}')
    z=$(awk -v a="$zA" -v b="$zB" -v t="$t" 'BEGIN{printf("%.6f", a + (b-a)*t)}')

    # viewport size (what the 'camera' sees before we resize back to original)
    vw=$(awk -v iw="$IMG_W" -v z="$z" 'BEGIN{printf("%d", iw / z)}')
    vh=$(awk -v ih="$IMG_H" -v z="$z" 'BEGIN{printf("%d", ih / z)}')

    # ensure integer and at least 1
    if [ "$vw" -lt 1 ]; then vw=1; fi
    if [ "$vh" -lt 1 ]; then vh=1; fi

    # compute top-left of crop, clamp to image boundaries
    x0=$(( cx - vw/2 ))
    y0=$(( cy - vh/2 ))
    if [ "$x0" -lt 0 ]; then x0=0; fi
    if [ "$y0" -lt 0 ]; then y0=0; fi
    if [ $(( x0 + vw )) -gt "$IMG_W" ]; then x0=$(( IMG_W - vw )); fi
    if [ $(( y0 + vh )) -gt "$IMG_H" ]; then y0=$(( IMG_H - vh )); fi

    frame_file="$TMPDIR/frame_$(printf "%05d" "$FRAME_INDEX").jpg"

    # crop then resize back to original dimensions (keeps a smooth zoom effect)
    # Using +repage to avoid page offsets. Using -filter Lanczos by default (ImageMagick)
    $USE_MAGICK "$IMG" -crop "${vw}x${vh}+${x0}+${y0}" +repage -resize "${IMG_W}x${IMG_H}" "$frame_file"

    # optionally, you can overlay a small marker or crosshair for debugging:
    # $USE_MAGICK "$frame_file" -stroke red -strokewidth 2 -draw "line $((IMG_W/2-10)),$((IMG_H/2)) $((IMG_W/2+10)),$((IMG_H/2))" "$frame_file"

    FRAME_INDEX=$(( FRAME_INDEX + 1 ))
    # print progress every 50 frames
    if [ $((FRAME_INDEX % 50)) -eq 0 ]; then echo "  generated frames: $FRAME_INDEX"; fi
  done
done

echo "Total frames generated: $FRAME_INDEX"

# Step 5: encode to MP4 using ffmpeg
echo "Encoding ${OUT} at ${FPS} fps..."
# Use libx264, yuv420p for wide compatibility
ffmpeg -y -framerate "$FPS" -i "$TMPDIR/frame_%05d.jpg" -c:v libx264 -pix_fmt yuv420p -vf "format=yuv420p" "$OUT" < /dev/null

echo "Done. Output: $OUT"
