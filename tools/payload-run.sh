#!/bin/sh
# run -- the payload's entry point under brenn-app.service.
#
# Started as `app` with the payload root as its working directory and TMPDIR
# naming the payload's scratch space. Empties the two log roots under scratch,
# copies the run's provenance and configuration beside where the records will
# land (starting nothing when a copy, or the directory the copies land in,
# fails), starts the library tour ahead of the harness launcher, and exits 0
# whatever the launcher returned: a restart onto whatever survived a crash
# would re-arm the bus five seconds after the control process fell over.
#
# The tour ends the run itself through the launcher's quit API; a tour that
# never ends is a launcher left serving on 8080, which is the state to look at.
# A tour still running ten seconds after the launcher has gone is stopped --
# TERM, then KILL ten seconds later -- because with no launcher there is
# nothing it can do but hold this service active.
[ -r "${BRENN_CA_FILE:-}" ] && export SSL_CERT_FILE="$BRENN_CA_FILE"
logs="${TMPDIR:?}/logs"
# Empty both roots first: the fetch and the analyzers read the newest run
# under them, so a run that wrote nothing would be judged on an earlier run's
# records. A wipe that fails is a root run's leftovers, which this account
# cannot remove; nothing is started, because a restart onto the same
# directories would say nothing new.
if ! rm -rf -- "$logs/motion" "$logs/launch" || ! mkdir -p -- "$logs/motion" "$logs/launch"; then
	echo "run: cannot empty $logs; a root run left directories here -- reboot"
	exit 0
fi
if [ -f provenance.txt ]; then
	cp -- provenance.txt "$logs/motion/" || { echo "run: the payload carries provenance.txt but copying it beside the records failed"; exit 0; }
fi
# The run's configuration beside its records: the files the analyzers read,
# at their payload-relative paths. Spelled here and in tools/lib.sh's
# run_config_files; tools/deploy-motion.test.sh holds the two lists equal.
mkdir -p -- "$logs/motion/config/cogs" || { echo "run: cannot make $logs/motion/config beside the records; nothing started"; exit 0; }
for f in cogs/servo_profile.textproto cogs/servo_gains.textproto cogs/mover_params.textproto; do
	cp -- "$f" "$logs/motion/config/$f" || { echo "run: no $f in the payload"; exit 0; }
done
ask='' launcher=''
# TERM to both children by PID, whether the signal came from systemd or from
# outside it. TERM and not INT: a child started with & by a non-interactive sh
# begins with SIGINT ignored, and reachy_motord de-torques on either.
# Invoked by the trap below, which the linter does not follow.
# shellcheck disable=SC2329
forward() {
	for p in "$launcher" "$ask"; do
		[ -n "$p" ] && kill -TERM "$p" 2>/dev/null
	done
}
trap forward TERM INT
./reachy_ask --tour cogs/library.names.json >"$logs/launch/reachy_ask.log" 2>&1 &
ask=$!
./simplelaunch robotcpu_harness.textproto --logdir "$logs/launch" &
launcher=$!
wait "$launcher"
rc=$?
# A trapped signal returns wait early with the launcher still winding down;
# wait again for its real status.
if kill -0 "$launcher" 2>/dev/null; then
	wait "$launcher"
	rc=$?
fi
# The tour ends the run by quitting the launcher, so once the launcher has
# gone the sender is already leaving. One that is not -- a launcher that
# fell over mid-tour, the sender left playing the plan into nothing --
# would hold this service active for the rest of the plan. Ten seconds to
# leave on its own, TERM (reachy_ask stops on it), ten more, then KILL.
i=0
while kill -0 "$ask" 2>/dev/null && [ "$i" -lt 10 ]; do
	sleep 1
	i=$((i + 1))
done
if kill -0 "$ask" 2>/dev/null; then
	echo "run: the tour was still running 10 s after the launcher; stopping it"
	kill -TERM "$ask" 2>/dev/null
	i=0
	while kill -0 "$ask" 2>/dev/null && [ "$i" -lt 10 ]; do
		sleep 1
		i=$((i + 1))
	done
	if kill -0 "$ask" 2>/dev/null; then
		kill -KILL "$ask" 2>/dev/null
	fi
fi
wait "$ask" 2>/dev/null
echo "run: launcher exited $rc"
exit 0
