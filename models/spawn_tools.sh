#!/usr/bin/env bash
# spawn_tools.sh — delete existing tool models, spawn all tools in a centred grid, nudge them into a slow spin.
#
# Usage (run from this script's folder):
#   ./spawn_tools.sh [FOLDER]                   # FOLDER = sub-folder holding the models (default: tools)
#   VARIANT=bright ./spawn_tools.sh models      # bright single-colour models (<name>_bright)
#   DRY_RUN=1 VARIANT=bright ./spawn_tools.sh models
# Env options:
#   VARIANT=default|bright   default-colour <name> (default) or bright <name>_bright models
#   ONLY=name[,name...]      spawn only these tools (base names, e.g. ONLY=dead_blow_hammer)
#   NUDGE=0                  skip the nudge step
#   DRY_RUN=1                print the commands instead of running them
#
# Mesh URIs are model://<name>/meshes/..., so the running gzserver/gzclient must have ./FOLDER
# itself in GAZEBO_MODEL_PATH (set before launching the sim). The script checks this and stops if not.

set -uo pipefail

FOLDER="${1:-tools}"
VARIANT="${VARIANT:-default}"
[[ "$VARIANT" == default || "$VARIANT" == bright ]] || { echo "ERROR: VARIANT must be 'default' or 'bright' (got '$VARIANT')" >&2; exit 1; }
MODELS_DIR="$(realpath "./$FOLDER")"
NUDGE="${NUDGE:-1}"
ONLY="${ONLY:-}"
DRY_RUN="${DRY_RUN:-0}"

# Grid: centred in this region, SPACING between tool origins [m]
X_MIN=10.3; X_MAX=11.6
Y_MIN=-9.7; Y_MAX=-3.6
Z_MIN=4.1;  Z_MAX=5.5
SPACING=0.5

# Nudge: per-tool torque = I * omega / DURATION, so every tool spins at OMEGA_DEG about each body axis
OMEGA_DEG=10.0     # deg/s per axis
DURATION=0.1       # s

run() { if [[ "$DRY_RUN" == 1 ]]; then echo "+ $*" >&2; else "$@"; fi; }

[[ -d "$MODELS_DIR" ]] || { echo "ERROR: models dir not found: $MODELS_DIR (run from the script's folder)" >&2; exit 1; }

# ---------- 0. sanity checks ----------
GZMP="${GAZEBO_MODEL_PATH:-}"
if [[ "$DRY_RUN" != 1 ]]; then
  rosservice list 2>/dev/null | grep -q '^/gazebo/spawn_sdf_model$' \
    || { echo "ERROR: Gazebo ROS services not available (is the sim running / ROS_MASTER_URI set?)" >&2; exit 1; }
  GZPID=$(pgrep -x gzserver | head -1)
  # the running server's path is what matters, not this shell's
  [[ -n "$GZPID" ]] && GZMP=$(tr '\0' '\n' < "/proc/$GZPID/environ" 2>/dev/null | sed -n 's/^GAZEBO_MODEL_PATH=//p')
fi
on_path() { local d; IFS=':' read -ra _P <<< "$GZMP"; for d in "${_P[@]}"; do [[ -n "$d" && "$(realpath -m "$d")" == "$1" ]] && return 0; done; return 1; }
if ! on_path "$MODELS_DIR"; then
  echo "ERROR: $MODELS_DIR is not in gzserver's GAZEBO_MODEL_PATH ($GZMP)." >&2
  echo "       Meshes would not resolve (invisible models, no collision). Before launching the sim:" >&2
  echo "       export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$MODELS_DIR" >&2
  [[ "$DRY_RUN" == 1 ]] || exit 1
fi

# ---------- 1. plan: name x y z tx ty tz ----------
PLAN=$(python3 - "$MODELS_DIR" "$VARIANT" "$ONLY" <<EOF
import sys, os, itertools, math, xml.etree.ElementTree as ET
d, variant = sys.argv[1], sys.argv[2]
names = sorted(n for n in os.listdir(d) if os.path.isfile(os.path.join(d, n, 'model.sdf')))
names = [n for n in names if n.endswith('_bright') == (variant == 'bright')]
only = [o.strip() for o in sys.argv[3].split(',') if o.strip()]
if only: names = [n for n in names if n.replace('_bright', '') in only]

def axis(lo, hi, s):
    n = int(math.floor((hi - lo) / s + 1e-9)) + 1
    c = (lo + hi) / 2
    return [c + (i - (n - 1) / 2) * s for i in range(n)]
X = axis($X_MIN, $X_MAX, $SPACING); Y = axis($Y_MIN, $Y_MAX, $SPACING); Z = axis($Z_MIN, $Z_MAX, $SPACING)
c = (($X_MIN + $X_MAX) / 2, ($Y_MIN + $Y_MAX) / 2, ($Z_MIN + $Z_MAX) / 2)
slots = sorted(itertools.product(X, Y, Z),
               key=lambda p: (round(sum((a - b) ** 2 for a, b in zip(p, c)), 6), p[2], p[1], p[0]))
if len(names) > len(slots):
    sys.exit(f'ERROR: {len(names)} models but only {len(slots)} grid slots')
w = math.radians($OMEGA_DEG); dt = $DURATION
for n, p in zip(names, slots):
    I = ET.parse(os.path.join(d, n, 'model.sdf')).getroot().find('.//inertia')
    t = [float(I.find(k).text) * w / dt for k in ('ixx', 'iyy', 'izz')]
    print(n, *(f'{v:.3f}' for v in p), *(f'{v:.4e}' for v in t))
EOF
) || { echo "$PLAN" >&2; exit 1; }
[[ -n "$PLAN" ]] || { echo "ERROR: no $VARIANT models found in $MODELS_DIR" >&2; exit 1; }
echo "Models: $(wc -l <<< "$PLAN")  (variant=$VARIANT)"

# ---------- 2. delete any tool models already in the world (both variants) ----------
EXISTING=""
if [[ "$DRY_RUN" != 1 ]]; then
  EXISTING=$(rosservice call /gazebo/get_world_properties 2>/dev/null \
    | python3 -c "import sys,yaml; print('\n'.join(yaml.safe_load(sys.stdin).get('model_names') or []))")
fi
ndel=0
for dir in "$MODELS_DIR"/*/; do
  name=$(basename "$dir")
  [[ -f "$dir/model.sdf" ]] || continue
  if grep -qxF "$name" <<< "$EXISTING"; then
    run rosservice call /gazebo/delete_model "{model_name: '$name'}" >/dev/null && ((ndel++))
  fi
done
echo "Deleted: $ndel existing tool model(s)"

# ---------- 3. spawn in grid, centre outwards ----------
nsp=0; nfail=0
while read -r name x y z _; do
  echo "spawn  $name  @ ($x, $y, $z)"
  if run rosrun gazebo_ros spawn_model -sdf -file "$MODELS_DIR/$name/model.sdf" -model "$name" \
        -x "$x" -y "$y" -z "$z" -R 0 -P 0 -Y 0 >/dev/null 2>&1; then
    ((nsp++))
  else
    echo "  FAILED: $name" >&2; ((nfail++))
  fi
done <<< "$PLAN"
echo "Spawned: $nsp  failed: $nfail"

# ---------- 4. nudge: pure torque about CoM (link origin = CoM; body axes = world axes at spawn) ----------
if [[ "$NUDGE" == 1 ]]; then
  echo "Nudging: ${OMEGA_DEG} deg/s per axis over ${DURATION}s"
  DS=$(python3 -c "print(int($DURATION))")
  DNS=$(python3 -c "print(int(round(($DURATION - int($DURATION)) * 1e9)))")
  while read -r name _ _ _ tx ty tz; do
    run rosservice call /gazebo/apply_body_wrench "body_name: '$name::link'
reference_frame: 'world'
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
  force: {x: 0.0, y: 0.0, z: 0.0}
  torque: {x: $tx, y: $ty, z: $tz}
start_time: {secs: 0, nsecs: 0}
duration: {secs: $DS, nsecs: $DNS}" >/dev/null || echo "  nudge FAILED: $name" >&2
  done <<< "$PLAN"
fi
echo "Done."
