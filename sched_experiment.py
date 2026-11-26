#!/usr/bin/env python3
"""
sched_experiment.py - scheduler preemption experiment for ledrobot (ledcontrol.sh)

IDEA
----
ledcontrol.sh talks to the STM32 through a serial port. It does not care whether a
real chip or a program is on the other end. So this tool:

  1. acts as a FAKE STM32: it creates a pseudo-terminal (a software serial port)
     and ledcontrol.sh is started UNMODIFIED against it instead of /dev/stm32;
  2. sends "hello" frames (CMD 0xFF) like the real board does at boot, and measures
     how long ledcontrol.sh needs to answer "LED ON" (CMD 0x01)  -> response latency;
  3. repeats that while N CPU-hog programs compete for the same CPU core, under
     different scheduling policies (SCHED_OTHER, nice, SCHED_RR, SCHED_FIFO);
  4. also reads the kernel's own counters for the script's process, so you can show
     the preemptions (involuntary context switches) and the time it waited for a CPU.

USAGE (root is needed for real-time policies and negative nice values)
    sudo python3 sched_experiment.py --script ./ledcontrol.sh --quick     # short test
    sudo python3 sched_experiment.py --script ./ledcontrol.sh             # full run
"""
import argparse
import csv
import math
import os
import platform
import pty
import select
import signal
import subprocess
import sys
import time
import tty

# ----------------------------------------------------------------------------
# Frame format (same as ledcontrol.sh): [FF FE ID CMD 00 00 00 CHK], CHK = XOR of bytes 0..6
# ----------------------------------------------------------------------------
H1, H2, STM_ID = 0xFF, 0xFE, 0x67
FRAME_LEN = 8
CMD_HELLO, CMD_LED_ON = 0xFF, 0x01


def make_frame(cmd):
    body = [H1, H2, STM_ID, cmd, 0, 0, 0]
    chk = 0
    for b in body:
        chk ^= b
    return bytes(body + [chk])


HELLO = make_frame(CMD_HELLO)


class FrameParser:
    """Same resync logic as rx_loop in ledcontrol.sh: find header, verify checksum."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        frames = []
        while len(self.buf) >= FRAME_LEN:
            if self.buf[0] != H1 or self.buf[1] != H2:
                del self.buf[0]
                continue
            chk = 0
            for b in self.buf[:FRAME_LEN - 1]:
                chk ^= b
            if chk != self.buf[FRAME_LEN - 1]:
                del self.buf[0]
                continue
            frames.append(bytes(self.buf[:FRAME_LEN]))
            del self.buf[:FRAME_LEN]
        return frames


# ----------------------------------------------------------------------------
# Fake STM32 = one end of a pseudo-terminal
# ----------------------------------------------------------------------------
class FakeSTM32:
    def __init__(self, link):
        self.link = link
        self.master, self.slave = pty.openpty()   # we keep BOTH ends open: pty never "hangs up"
        tty.setraw(self.slave)                    # raw mode, no echo (ledcontrol.sh sets it again)
        if os.path.lexists(link):
            os.unlink(link)
        os.symlink(os.ttyname(self.slave), link)  # e.g. /tmp/fake_stm32 -> /dev/pts/7
        self.parser = FrameParser()

    def close(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
        if os.path.islink(self.link):
            os.unlink(self.link)

    def _read(self, timeout_s):
        """Return (frames, timestamp_ns_right_after_read)."""
        ready, _, _ = select.select([self.master], [], [], max(0.0, timeout_s))
        if not ready:
            return [], None
        try:
            data = os.read(self.master, 4096)
        except OSError:
            return [], None
        t = time.monotonic_ns()
        return self.parser.feed(data), t

    @staticmethod
    def _is_reply(frame):
        return frame[2] == STM_ID and frame[3] == CMD_LED_ON

    def wait_ready(self, timeout_s):
        """ledcontrol.sh sends one 'LED ON' by itself ~0.3 s after it has configured the port."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            frames, _ = self._read(0.1)
            if any(self._is_reply(f) for f in frames):
                return True
        return False

    def drain(self):
        while True:
            ready, _, _ = select.select([self.master], [], [], 0)
            if not ready:
                break
            try:
                os.read(self.master, 4096)
            except OSError:
                break
        self.parser.buf.clear()

    def trial(self, timeout_s):
        """Send one hello, return response latency in ms (None if no reply in time)."""
        self.drain()
        t0 = time.monotonic_ns()
        os.write(self.master, HELLO)
        end = time.monotonic() + timeout_s
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                return None
            frames, t = self._read(remaining)
            if any(self._is_reply(f) for f in frames):
                return (t - t0) / 1e6


# ----------------------------------------------------------------------------
# Experimental conditions: (policy, level). level = priority for rr/fifo, nice for other.
#   "sut" = the system under test (ledcontrol.sh), "hog" = the competing CPU hogs.
# ----------------------------------------------------------------------------
CONDITIONS = {
    "other":   dict(sut=("other", 0),   hog=("other", 0), desc="SCHED_OTHER, nice 0 (Linux default)"),
    "nice-10": dict(sut=("other", -10), hog=("other", 0), desc="SCHED_OTHER, nice -10 (higher weight)"),
    "rr-50":   dict(sut=("rr", 50),     hog=("other", 0), desc="SCHED_RR prio 50 vs normal hogs"),
    "fifo-50": dict(sut=("fifo", 50),   hog=("other", 0), desc="SCHED_FIFO prio 50 vs normal hogs"),
    # Same priority -> classic round robin: every time the script blocks (fork, wait, read) it
    # goes to the back of the queue and waits (#hogs x 100 ms). One frame then takes SECONDS,
    # so this condition uses fewer frames.
    "rr-eq":   dict(sut=("rr", 50),     hog=("rr", 50),   frames=5, warmup=1,
                    desc="SCHED_RR prio 50 vs RR hogs prio 50 (same priority: round-robin slicing)"),
    "rr-hi":   dict(sut=("rr", 60),     hog=("rr", 50),   desc="SCHED_RR prio 60 vs RR hogs prio 50 (higher priority preempts)"),
}


def needs_rt(cond):
    return cond["sut"][0] != "other" or cond["hog"][0] != "other" or cond["sut"][1] < 0


def make_preexec(cpu, policy, level):
    """Runs in the child right after fork, before exec -> policy/affinity are inherited by
    everything the child starts later (for ledcontrol.sh that includes every dd and od)."""
    def fn():
        os.sched_setaffinity(0, {cpu})
        if policy == "rr":
            os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(level))
        elif policy == "fifo":
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(level))
        else:
            os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
            os.setpriority(os.PRIO_PROCESS, 0, level)
    return fn


# CPU hog: pure busy loop, ends by itself after N seconds (safety net if this tool crashes)
HOG_CODE = (
    "import sys, time\n"
    "end = time.monotonic() + float(sys.argv[1])\n"
    "while time.monotonic() < end:\n"
    "    pass\n"
)


# ----------------------------------------------------------------------------
# Kernel counters for the script's long-lived receive process
# ----------------------------------------------------------------------------
def children_of(pid):
    out = []
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/stat") as f:
                stat = f.read()
            ppid = int(stat.rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            continue
        if ppid == pid:
            out.append(int(d))
    return sorted(out)


def read_counters(pid):
    """involuntary (= preempted) context switches and time spent waiting on the run queue."""
    try:
        invol = None
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("nonvoluntary_ctxt_switches"):
                    invol = int(line.split()[1])
        wait_ns = None
        try:
            with open(f"/proc/{pid}/schedstat") as f:
                wait_ns = int(f.read().split()[1])
        except (OSError, IndexError, ValueError):
            pass
        return dict(invol=invol, wait_ns=wait_ns)
    except OSError:
        return None


# ----------------------------------------------------------------------------
# One run = one (condition, load, repetition)
# ----------------------------------------------------------------------------
def run_once(args, cond, load):
    res = dict(lat=[], aborted=False, invol=None, wait_ms=None)
    dev = FakeSTM32(args.link)
    hogs, sut = [], None
    try:
        hog_secs = args.max_run_s + args.ready_timeout + 30
        for _ in range(load):
            hogs.append(subprocess.Popen(
                [sys.executable, "-c", HOG_CODE, str(hog_secs)],
                preexec_fn=make_preexec(args.test_cpu, *cond["hog"]),
                stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

        sut = subprocess.Popen(
            ["/bin/bash", args.script, args.link, "115200"],
            preexec_fn=make_preexec(args.test_cpu, *cond["sut"]),
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)   # stdin must NOT be a tty, else the script enters manual mode

        if not dev.wait_ready(args.ready_timeout):
            res["aborted"] = True
            return res

        kids = children_of(sut.pid)
        rx_pid = kids[0] if kids else None
        before = read_counters(rx_pid) if rx_pid else None

        warmup = min(args.warmup, cond.get("warmup", args.warmup))
        frames = min(args.frames, cond.get("frames", args.frames))
        for _ in range(warmup):                      # first frames are "cold": discard
            if dev.trial(args.timeout_s) is None:
                res["aborted"] = True
                return res

        t_end = time.monotonic() + args.max_run_s
        for _ in range(frames):
            if time.monotonic() > t_end:
                break
            lat = dev.trial(args.timeout_s)
            if lat is None:
                res["aborted"] = True
                break
            res["lat"].append(lat)
            time.sleep(args.gap_ms / 1000.0)

        after = read_counters(rx_pid) if rx_pid else None
        if before and after:
            if before["invol"] is not None and after["invol"] is not None:
                res["invol"] = after["invol"] - before["invol"]
            if before["wait_ns"] is not None and after["wait_ns"] is not None:
                res["wait_ms"] = (after["wait_ns"] - before["wait_ns"]) / 1e6
        return res
    finally:
        for h in hogs:                               # hogs first, so the script can exit quickly
            h.kill()
        for h in hogs:
            h.wait()
        if sut is not None:
            try:
                os.killpg(sut.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                sut.wait(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(sut.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                sut.wait()
        dev.close()


# ----------------------------------------------------------------------------
# Statistics and output
# ----------------------------------------------------------------------------
def percentile(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return sorted_vals[int(k)]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def mean(v):
    return sum(v) / len(v) if v else float("nan")


def stdev(v):
    if len(v) < 2:
        return float("nan")
    m = mean(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def summarize(results, deadline_ms):
    rows = []
    for (cond, load), d in results.items():
        lat = sorted(d["lat"])
        over = sum(1 for x in lat if x > deadline_ms)
        rows.append(dict(
            condition=cond, load=load, runs=d["runs"], aborted_runs=d["aborted"], n=len(lat),
            mean_ms=mean(lat), median_ms=percentile(lat, 50), p95_ms=percentile(lat, 95),
            p99_ms=percentile(lat, 99), max_ms=lat[-1] if lat else float("nan"),
            stdev_ms=stdev(lat),
            over_deadline_pct=100.0 * over / len(lat) if lat else float("nan"),
            invol_ctxt_per_run=mean(d["invol"]), rq_wait_ms_per_run=mean(d["wait_ms"])))
    return rows


def print_table(rows, deadline_ms):
    hdr = (f"{'condition':<9} {'load':>4} {'n':>5} {'mean':>8} {'median':>8} {'p95':>8} {'p99':>8} "
           f"{'max':>8} {'stdev':>8} {'>'+str(int(deadline_ms))+'ms%':>8} {'abort':>5} "
           f"{'preempt':>8} {'rqwait':>8}")
    print("\nResponse latency in ms (preempt = involuntary context switches per run, "
          "rqwait = ms spent waiting for CPU per run)")
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(f"{r['condition']:<9} {r['load']:>4} {r['n']:>5} {r['mean_ms']:>8.1f} {r['median_ms']:>8.1f} "
              f"{r['p95_ms']:>8.1f} {r['p99_ms']:>8.1f} {r['max_ms']:>8.1f} {r['stdev_ms']:>8.1f} "
              f"{r['over_deadline_pct']:>8.1f} {r['aborted_runs']:>5} "
              f"{r['invol_ctxt_per_run']:>8.0f} {r['rq_wait_ms_per_run']:>8.0f}")


def read_sysctl(name):
    try:
        with open(f"/proc/sys/kernel/{name}") as f:
            return f.read().strip()
    except OSError:
        return "n/a"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--script", default="./ledcontrol.sh", help="path to ledcontrol.sh (system under test)")
    ap.add_argument("--link", default="/tmp/fake_stm32", help="path of the fake serial port")
    ap.add_argument("--loads", default="0,2,4", help="numbers of competing CPU-hog programs")
    ap.add_argument("--conditions", default="all", help="comma list of: " + ",".join(CONDITIONS) + " (or 'all')")
    ap.add_argument("--reps", type=int, default=3, help="repetitions of every (condition, load)")
    ap.add_argument("--frames", type=int, default=50, help="measured hello frames per run")
    ap.add_argument("--warmup", type=int, default=3, help="discarded hello frames at start of a run")
    ap.add_argument("--gap-ms", type=float, default=50, help="pause between two hello frames")
    ap.add_argument("--deadline-ms", type=float, default=50, help="a reply slower than this counts as a deadline miss")
    ap.add_argument("--timeout-s", type=float, default=90, help="no reply after this long -> run aborted")
    ap.add_argument("--max-run-s", type=float, default=120, help="hard time limit of the measured part of a run")
    ap.add_argument("--ready-timeout", type=float, default=60, help="time allowed for the script to start up")
    ap.add_argument("--test-cpu", type=int, help="CPU core for the script and hogs (default: first allowed)")
    ap.add_argument("--ctl-cpu", type=int, help="CPU core for this tool (default: last allowed)")
    ap.add_argument("--no-rt", action="store_true", help="no real-time policies: skips rr/fifo/nice-10 conditions")
    ap.add_argument("--outdir", default="results")
    ap.add_argument("--quick", action="store_true", help="short smoke test (1 rep, 15 frames)")
    args = ap.parse_args()

    if args.quick:
        args.reps, args.frames, args.max_run_s = 1, 15, 30
    if not os.path.isfile(args.script):
        sys.exit(f"Script not found: {args.script}")

    allowed = sorted(os.sched_getaffinity(0))
    args.test_cpu = allowed[0] if args.test_cpu is None else args.test_cpu
    args.ctl_cpu = (allowed[-1] if len(allowed) > 1 else allowed[0]) if args.ctl_cpu is None else args.ctl_cpu

    names = list(CONDITIONS) if args.conditions == "all" else [c.strip() for c in args.conditions.split(",")]
    for n in names:
        if n not in CONDITIONS:
            sys.exit(f"Unknown condition '{n}'. Choose from: {', '.join(CONDITIONS)}")
    if args.no_rt:
        skipped = [n for n in names if needs_rt(CONDITIONS[n])]
        names = [n for n in names if n not in skipped]
        if skipped:
            print(f"--no-rt: skipping {', '.join(skipped)}")
    loads = [int(x) for x in args.loads.split(",")]

    # This tool (the fake STM32 + stopwatch) must not be disturbed by the load it creates:
    # own CPU core when available, and top real-time priority.
    os.sched_setaffinity(0, {args.ctl_cpu})
    if not args.no_rt:
        if os.geteuid() != 0:
            sys.exit("Run with sudo (real-time policies / negative nice need root), or use --no-rt.")
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(90))
        except OSError as e:
            sys.exit(f"Real-time scheduling not allowed here ({e}). Try --no-rt, or run on the host/VM.")

    info = [f"kernel            : {platform.release()}",
            f"allowed CPUs      : {allowed}  (script+hogs on CPU {args.test_cpu}, this tool on CPU {args.ctl_cpu})",
            f"RR time slice     : {read_sysctl('sched_rr_timeslice_ms')} ms",
            f"RT throttle       : {read_sysctl('sched_rt_runtime_us')} us per {read_sysctl('sched_rt_period_us')} us",
            f"frames/run        : {args.frames} (+{args.warmup} warm-up), gap {args.gap_ms} ms, deadline {args.deadline_ms} ms",
            f"conditions        : {', '.join(names)}",
            f"loads (hogs)      : {loads}, repetitions {args.reps}"]
    print("\n".join(info))
    os.makedirs(args.outdir, exist_ok=True)
    with open(os.path.join(args.outdir, "run_info.txt"), "w") as f:
        f.write("\n".join(info) + "\n")

    # load 0 with RT hogs is identical to load 0 with normal hogs -> skip duplicates
    plan = [(n, l) for n in names for l in loads if not (l == 0 and CONDITIONS[n]["hog"][0] != "other")]
    results = {k: dict(lat=[], aborted=0, runs=0, invol=[], wait_ms=[]) for k in plan}

    raw_path = os.path.join(args.outdir, "results_raw.csv")
    try:
        with open(raw_path, "w", newline="") as raw:
            w = csv.writer(raw)
            w.writerow(["condition", "load", "rep", "seq", "latency_ms"])
            for rep in range(1, args.reps + 1):          # reps interleaved: slow drifts hit all conditions equally
                for name, load in plan:
                    r = run_once(args, CONDITIONS[name], load)
                    d = results[(name, load)]
                    d["runs"] += 1
                    d["aborted"] += int(r["aborted"])
                    d["lat"] += r["lat"]
                    if r["invol"] is not None:
                        d["invol"].append(r["invol"])
                    if r["wait_ms"] is not None:
                        d["wait_ms"].append(r["wait_ms"])
                    for i, x in enumerate(r["lat"], 1):
                        w.writerow([name, load, rep, i, f"{x:.3f}"])
                    raw.flush()
                    avg = f"{mean(r['lat']):.1f} ms" if r["lat"] else "-"
                    mx = f"{max(r['lat']):.1f} ms" if r["lat"] else "-"
                    print(f"[rep {rep}/{args.reps}] {name:<8} load={load}  frames={len(r['lat']):>3}  "
                          f"mean={avg}  max={mx}" + ("  ABORTED" if r["aborted"] else ""))
    except KeyboardInterrupt:
        print("\nInterrupted - summarising what was collected so far.")

    rows = summarize(results, args.deadline_ms)
    print_table(rows, args.deadline_ms)
    with open(os.path.join(args.outdir, "results_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nSaved: {raw_path}, results_summary.csv, run_info.txt in {args.outdir}/")


if __name__ == "__main__":
    main()