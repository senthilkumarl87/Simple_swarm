#!/usr/bin/env python3
"""
Swarm GUI — scenario browser + 3D preview + one-click PX4/MAVSDK simulate.

Left pane:
  * list of scenarios under Simple_swarm/test_inputs/
  * scenario summary (drone count, duration, description from meta.json)
  * simulation parameters (takeoff altitude, control rate, duration, dry-run)
  * Simulate / Stop / Refresh buttons

Right pane:
  * 3D plot of the leader path with the formation rendered at three samples
    (t=start, t=mid, t=end), each rotated by the leader heading at that
    sample — so the "formation rotates with leader heading" invariant is
    visible before you fly it.

Bottom pane:
  * streaming log of the orchestration (PX4 boot, mavsdk_server, controller).

Simulate button, per click:
  1. pkill stale px4 / mavsdk_server processes
  2. bash start_multi_px4.sh N     (N = drones in config)
  3. wait for PX4 SITL boot
  4. spawn N mavsdk_server processes  (gRPC 50041+, udp://:14540+)
  5. launch realtime_swarm_mavsdk.py with the selected scenario
  6. stream its stdout into the log pane

Stop button: kills the controller + every spawned PX4 / mavsdk_server.
"""

import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

import numpy as np
import pandas as pd
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# ---------------------------------------------------------------- paths -----
SIMPLE_SWARM_DIR = Path(__file__).resolve().parent
TEST_INPUTS_DIR  = SIMPLE_SWARM_DIR / "test_inputs"
LAUNCH_SCRIPT    = SIMPLE_SWARM_DIR / "start_multi_px4.sh"
CONTROLLER_SCRIPT = SIMPLE_SWARM_DIR / "realtime_swarm_mavsdk.py"
MAVSDK_SERVER    = SIMPLE_SWARM_DIR.parent / "mavsdk_server"
OUTPUT_DIR       = SIMPLE_SWARM_DIR / "output" / "mavsdk_realtime"

PX4_BOOT_WAIT_S   = 30.0   # SIH needs ~15s boot + EKF convergence; add headroom
MAVSDK_SETTLE_S   = 3.0
MAVSDK_GRPC_BASE  = 50040   # port = base + hw_id, matches realtime_swarm_mavsdk.py

# Every line written to the GUI log pane is also appended here, so errors that
# scroll off the pane (or that the user can't select/copy) can still be shared.
LOG_FILE = Path("/tmp/swarm_gui.log")


# ------------------------------------------------------- scenario loading ---

def list_scenarios() -> list[Path]:
    if not TEST_INPUTS_DIR.is_dir():
        return []
    return sorted(p for p in TEST_INPUTS_DIR.iterdir()
                  if p.is_dir() and (p / "drones_config.json").exists()
                  and (p / "formation.csv").exists()
                  and (p / "leader_path.csv").exists())


def load_scenario(path: Path):
    cfg = json.loads((path / "drones_config.json").read_text())
    leader_id = next(int(d["hw_id"]) for d in cfg["drones"] if d.get("is_leader"))
    formation = pd.read_csv(path / "formation.csv")
    leader_path = pd.read_csv(path / "leader_path.csv").sort_values("t").reset_index(drop=True)
    meta = {}
    mp = path / "meta.json"
    if mp.exists():
        try:
            meta = json.loads(mp.read_text())
        except Exception:
            meta = {}
    # if leader_path.csv has no heading, derive it from motion tangent
    if "heading" not in leader_path.columns:
        dn = np.gradient(leader_path["north"].to_numpy(), leader_path["t"].to_numpy())
        de = np.gradient(leader_path["east"].to_numpy(),  leader_path["t"].to_numpy())
        leader_path["heading"] = np.degrees(np.arctan2(de, dn))
    return {
        "path":        path,
        "cfg":         cfg,
        "leader_id":   leader_id,
        "n_drones":    len(cfg["drones"]),
        "formation":   formation,
        "leader_path": leader_path,
        "meta":        meta,
    }


def rotate_body_to_ned(on, oe, heading_deg):
    th = np.radians(heading_deg)
    c, s = np.cos(th), np.sin(th)
    return on * c - oe * s, on * s + oe * c


# ------------------------------------------------------------------- GUI ----

class SwarmGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Simple_swarm — Scenario Browser + PX4/MAVSDK Simulator")
        self.root.geometry("1280x820")

        self.scenarios: list[Path] = list_scenarios()
        self.current: dict | None = None
        self.proc_px4_launcher: subprocess.Popen | None = None
        self.proc_mavsdk: list[subprocess.Popen] = []
        self.proc_controller: subprocess.Popen | None = None
        self.sim_thread: threading.Thread | None = None
        self.sim_cancel = threading.Event()
        self.log_queue: queue.Queue = queue.Queue()

        # Open the disk log (truncate at GUI start so each session is clean).
        try:
            self._log_fh = LOG_FILE.open("w", buffering=1)   # line-buffered
        except Exception as e:
            print(f"[gui] could not open {LOG_FILE}: {e}", file=sys.stderr)
            self._log_fh = None

        # --- top-level grid ---
        self.root.columnconfigure(0, weight=0, minsize=320)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=3)
        self.root.rowconfigure(1, weight=1)

        self._build_left_pane()
        self._build_plot_pane()
        self._build_log_pane()

        # drain log_queue on UI thread
        self.root.after(100, self._drain_log)

        # clean shutdown if the window is closed mid-sim
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._log(f"[gui] session started; log mirrored to {LOG_FILE}\n")

        if self.scenarios:
            self.listbox.selection_set(0)
            self._on_select()

    # ------------------------------------------------ left pane (controls) --
    def _build_left_pane(self):
        frame = ttk.Frame(self.root, padding=8)
        frame.grid(row=0, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)

        ttk.Label(frame, text="Scenarios", font=("TkDefaultFont", 11, "bold")).grid(
            row=0, column=0, sticky="w")

        self.listbox = tk.Listbox(frame, height=10, exportselection=False)
        for p in self.scenarios:
            self.listbox.insert("end", p.name)
        self.listbox.grid(row=1, column=0, sticky="nsew", pady=(2, 6))
        self.listbox.bind("<<ListboxSelect>>", lambda _e: self._on_select())
        frame.rowconfigure(1, weight=1)

        ttk.Button(frame, text="Refresh list", command=self._refresh).grid(
            row=2, column=0, sticky="ew", pady=(0, 8))

        ttk.Separator(frame).grid(row=3, column=0, sticky="ew", pady=4)

        # scenario summary
        ttk.Label(frame, text="Summary", font=("TkDefaultFont", 11, "bold")).grid(
            row=4, column=0, sticky="w")
        self.summary = tk.Text(frame, height=8, width=36, wrap="word",
                               state="disabled", relief="flat")
        self.summary.grid(row=5, column=0, sticky="ew", pady=(2, 6))

        ttk.Separator(frame).grid(row=6, column=0, sticky="ew", pady=4)

        # parameters
        ttk.Label(frame, text="Simulation parameters",
                  font=("TkDefaultFont", 11, "bold")).grid(row=7, column=0, sticky="w")

        pf = ttk.Frame(frame)
        pf.grid(row=8, column=0, sticky="ew", pady=(2, 6))
        pf.columnconfigure(1, weight=1)

        self.var_takeoff_alt  = tk.DoubleVar(value=15.0)
        self.var_control_rate = tk.DoubleVar(value=20.0)
        self.var_duration     = tk.StringVar(value="")           # blank = match leader_path
        self.var_dry_run      = tk.BooleanVar(value=False)
        self.var_land_on_exit = tk.BooleanVar(value=True)

        def row(i, label, var, width=8):
            ttk.Label(pf, text=label).grid(row=i, column=0, sticky="w", padx=(0, 6))
            ttk.Entry(pf, textvariable=var, width=width).grid(row=i, column=1, sticky="ew")

        row(0, "Takeoff alt (m)", self.var_takeoff_alt)
        row(1, "Control rate (Hz)", self.var_control_rate)
        row(2, "Duration (s, blank=auto)", self.var_duration)
        ttk.Checkbutton(pf, text="Dry-run (no arm/offboard)",
                        variable=self.var_dry_run).grid(row=3, column=0, columnspan=2,
                                                         sticky="w", pady=(4, 0))
        ttk.Checkbutton(pf, text="Land on exit",
                        variable=self.var_land_on_exit).grid(row=4, column=0, columnspan=2,
                                                              sticky="w")

        # buttons
        bf = ttk.Frame(frame)
        bf.grid(row=9, column=0, sticky="ew", pady=(8, 0))
        bf.columnconfigure(0, weight=1)
        bf.columnconfigure(1, weight=1)
        self.btn_sim  = ttk.Button(bf, text="Simulate ▶",  command=self._on_simulate)
        self.btn_stop = ttk.Button(bf, text="Stop ■",      command=self._on_stop, state="disabled")
        self.btn_sim.grid(row=0,  column=0, sticky="ew", padx=(0, 4))
        self.btn_stop.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        # status
        self.status = tk.StringVar(value="Idle")
        ttk.Label(frame, textvariable=self.status, foreground="#555").grid(
            row=10, column=0, sticky="w", pady=(6, 0))

    # ------------------------------------------------- right pane (3D plot) --
    def _build_plot_pane(self):
        frame = ttk.Frame(self.root, padding=4)
        frame.grid(row=0, column=1, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax  = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        toolbar_frame = ttk.Frame(frame)
        toolbar_frame.grid(row=1, column=0, sticky="ew")
        NavigationToolbar2Tk(self.canvas, toolbar_frame)

    # -------------------------------------------------- bottom pane (log) ---
    def _build_log_pane(self):
        frame = ttk.Frame(self.root, padding=6)
        frame.grid(row=1, column=0, columnspan=2, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        header = ttk.Frame(frame)
        header.grid(row=0, column=0, columnspan=2, sticky="ew")
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text="Orchestration log",
                  font=("TkDefaultFont", 10, "bold")).grid(row=0, column=0, sticky="w")
        ttk.Label(header, text=f"(tee → {LOG_FILE})",
                  foreground="#888").grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Button(header, text="Copy log path",
                   command=self._copy_log_path).grid(row=0, column=2, sticky="e", padx=4)
        ttk.Button(header, text="Clear",
                   command=self._clear_log).grid(row=0, column=3, sticky="e")

        self.log = tk.Text(frame, height=10, wrap="none", background="#111",
                           foreground="#ddd", insertbackground="#ddd")
        self.log.grid(row=1, column=0, sticky="nsew")
        sb = ttk.Scrollbar(frame, orient="vertical", command=self.log.yview)
        sb.grid(row=1, column=1, sticky="ns")
        self.log.configure(yscrollcommand=sb.set)

    def _copy_log_path(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(str(LOG_FILE))
        self.status.set(f"Copied log path: {LOG_FILE}")

    def _clear_log(self):
        self.log.delete("1.0", "end")

    # -------------------------------------------------- scenario selection --
    def _refresh(self):
        sel = self.listbox.curselection()
        prev = self.scenarios[sel[0]].name if sel else None
        self.scenarios = list_scenarios()
        self.listbox.delete(0, "end")
        for p in self.scenarios:
            self.listbox.insert("end", p.name)
        if prev:
            for i, p in enumerate(self.scenarios):
                if p.name == prev:
                    self.listbox.selection_set(i)
                    break
        if self.listbox.curselection():
            self._on_select()

    def _on_select(self):
        sel = self.listbox.curselection()
        if not sel:
            return
        path = self.scenarios[sel[0]]
        try:
            self.current = load_scenario(path)
        except Exception as e:
            messagebox.showerror("Load failed", f"{path.name}: {e}")
            return
        self._update_summary()
        self._draw_scenario()

    def _update_summary(self):
        c = self.current
        lp = c["leader_path"]
        lines = [
            f"Name: {c['path'].name}",
            f"Drones: {c['n_drones']}  (leader hw_id={c['leader_id']})",
            f"Followers: {sorted(int(x) for x in c['formation']['hw_id'] if int(x) != c['leader_id'])}",
            f"Leader path: {len(lp)} samples, duration {lp['t'].iloc[-1]:.1f}s",
            f"Altitude range: {lp['altitude'].min():.1f}–{lp['altitude'].max():.1f} m",
        ]
        if c["meta"].get("description"):
            lines.append("")
            lines.append(f"Description: {c['meta']['description']}")
        self.summary.configure(state="normal")
        self.summary.delete("1.0", "end")
        self.summary.insert("1.0", "\n".join(lines))
        self.summary.configure(state="disabled")

    # -------------------------------------------------------- 3D rendering --
    def _draw_scenario(self):
        c = self.current
        lp = c["leader_path"]
        form = c["formation"]
        leader_id = c["leader_id"]

        self.ax.clear()

        # leader path
        ln = lp["north"].to_numpy()
        le = lp["east"].to_numpy()
        lu = lp["altitude"].to_numpy()
        self.ax.plot(ln, le, lu, color="#ffcc00", lw=1.5, label="leader path", alpha=0.85)
        self.ax.scatter([ln[0]], [le[0]], [lu[0]], c="#ffcc00", s=60, marker="^",
                        edgecolors="k", label="leader start")

        # sample the formation at 3 times (start, mid, end)
        n_samples = len(lp)
        sample_idx = [0, n_samples // 2, n_samples - 1]
        sample_colors = ["#00cc99", "#3399ff", "#ff4477"]
        sample_labels = ["t=start", "t=mid", "t=end"]

        # followers only — leader is at (ln,le,lu)
        followers = form[form["follow"] == leader_id]
        for si, color, lbl in zip(sample_idx, sample_colors, sample_labels):
            hdg = float(lp["heading"].iloc[si])
            Ln, Le, Lu = float(ln[si]), float(le[si]), float(lu[si])

            xs, ys, zs = [Ln], [Le], [Lu]   # include leader so polygon closes
            for _, row in followers.iterrows():
                on, oe, oalt = float(row["offset_n"]), float(row["offset_e"]), float(row["offset_alt"])
                rn, re = rotate_body_to_ned(on, oe, hdg)
                xs.append(Ln + rn)
                ys.append(Le + re)
                zs.append(Lu + oalt)

            self.ax.scatter(xs[0], ys[0], zs[0], c=color, s=90, marker="*",
                            edgecolors="k", label=f"leader @ {lbl} (hdg {hdg:+.0f}°)")
            self.ax.scatter(xs[1:], ys[1:], zs[1:], c=color, s=50, marker="o",
                            edgecolors="k")
            # spokes leader→each follower
            for fx, fy, fz in zip(xs[1:], ys[1:], zs[1:]):
                self.ax.plot([xs[0], fx], [ys[0], fy], [zs[0], fz],
                             color=color, lw=1.2, alpha=0.7)

        self.ax.set_xlabel("North (m)")
        self.ax.set_ylabel("East (m)")
        self.ax.set_zlabel("Altitude (m, up)")
        self.ax.set_title(f"{c['path'].name} — formation rotated by leader heading")

        # equal-ish aspect
        all_n = np.concatenate([ln, [ln[0]]])
        all_e = np.concatenate([le, [le[0]]])
        pad = 8.0
        self.ax.set_xlim(all_n.min() - pad, all_n.max() + pad)
        self.ax.set_ylim(all_e.min() - pad, all_e.max() + pad)
        self.ax.set_zlim(max(0.0, lu.min() - 5.0), lu.max() + 5.0)
        self.ax.legend(loc="upper left", fontsize=8, framealpha=0.85)
        self.canvas.draw_idle()

    # ------------------------------------------------- simulation controls --
    def _on_simulate(self):
        if self.current is None:
            messagebox.showwarning("No scenario", "Pick a scenario first.")
            return
        if self.sim_thread and self.sim_thread.is_alive():
            messagebox.showinfo("Already running", "A simulation is already running. Stop it first.")
            return
        if not LAUNCH_SCRIPT.exists():
            messagebox.showerror("Missing file", f"Launcher not found: {LAUNCH_SCRIPT}")
            return
        if not MAVSDK_SERVER.exists():
            messagebox.showerror("Missing file",
                                 f"mavsdk_server binary not found at {MAVSDK_SERVER}.\n"
                                 "Run: python3 setup_mavsdk_server.py")
            return

        self.sim_cancel.clear()
        self.btn_sim.configure(state="disabled")
        self.btn_stop.configure(state="normal")
        self.status.set("Launching …")
        self.sim_thread = threading.Thread(target=self._simulation_worker, daemon=True)
        self.sim_thread.start()

    def _on_stop(self):
        self.sim_cancel.set()
        self._log("\n[gui] stop requested — killing processes\n")
        self._kill_all()
        self.status.set("Stopped")
        self.btn_sim.configure(state="normal")
        self.btn_stop.configure(state="disabled")

    # ---------------------------------------------- worker-thread plumbing --
    def _simulation_worker(self):
        try:
            c = self.current
            n = c["n_drones"]
            scenario_path = c["path"]

            self._log(f"\n=== Simulate: {scenario_path.name} ({n} drones) ===\n")

            self._log("[1/5] killing any stale px4 / mavsdk_server ...\n")
            self._kill_stale_only()
            time.sleep(1.0)
            if self.sim_cancel.is_set(): return

            self._log(f"[2/5] launching {n} PX4 SITL instance(s) via start_multi_px4.sh ...\n")
            self.proc_px4_launcher = subprocess.Popen(
                ["bash", str(LAUNCH_SCRIPT), str(n)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                preexec_fn=os.setsid)
            # start_multi_px4.sh pipes progress lines as it spawns each PX4 terminal
            threading.Thread(target=self._pump_stream,
                             args=(self.proc_px4_launcher.stdout, "[px4-launcher] "),
                             daemon=True).start()

            # wait for boot (launcher delays 5s between drones, PX4 itself needs ~10s)
            boot_wait = PX4_BOOT_WAIT_S + 5 * max(0, n - 1)
            self._log(f"[3/5] waiting {boot_wait:.0f}s for PX4 SITL to boot ...\n")
            if self._interruptible_sleep(boot_wait): return

            self._log(f"[4/5] spawning {n} mavsdk_server(s) ...\n")
            for drone in c["cfg"]["drones"]:
                hw_id = int(drone["hw_id"])
                port  = MAVSDK_GRPC_BASE + hw_id                 # 50041, 50042, ...
                udp   = int(drone.get("port", 14540 + (hw_id - 1)))   # 14540, 14541, ...
                log_path = Path("/tmp") / f"mavsdk_server_{hw_id}.log"
                self._log(f"    hw_id {hw_id}: grpc={port}  mavlink=udp://:{udp}  log={log_path}\n")
                fh = log_path.open("w")
                p = subprocess.Popen(
                    [str(MAVSDK_SERVER), "-p", str(port), f"udp://:{udp}"],
                    stdout=fh, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
                self.proc_mavsdk.append(p)

            if self._interruptible_sleep(MAVSDK_SETTLE_S): return

            self._log("[5/5] launching realtime_swarm_mavsdk.py controller ...\n")
            cmd = [
                sys.executable, "-u", str(CONTROLLER_SCRIPT),
                "--config",        str(scenario_path / "drones_config.json"),
                "--formation",     str(scenario_path / "formation.csv"),
                "--leader-source", "csv",
                "--leader-path",   str(scenario_path / "leader_path.csv"),
                "--control-rate",  f"{float(self.var_control_rate.get())}",
                "--takeoff-alt",   f"{float(self.var_takeoff_alt.get())}",
                "--output-dir",    str(OUTPUT_DIR),
            ]
            dur = self.var_duration.get().strip()
            if dur:
                cmd += ["--duration", dur]
            else:
                # match the leader path duration so the run ends cleanly
                auto_dur = float(c["leader_path"]["t"].iloc[-1])
                cmd += ["--duration", f"{auto_dur:.1f}"]
            if self.var_dry_run.get():
                cmd += ["--dry-run"]
            if self.var_land_on_exit.get() and not self.var_dry_run.get():
                cmd += ["--land-on-exit"]

            self._log("    " + " ".join(cmd) + "\n")
            self.proc_controller = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, preexec_fn=os.setsid)
            self._set_status("Controller running")
            threading.Thread(target=self._pump_stream,
                             args=(self.proc_controller.stdout, "[controller] "),
                             daemon=True).start()

            rc = self.proc_controller.wait()
            self._log(f"[done] controller exited with rc={rc}\n")
            self._set_status(f"Finished (rc={rc})")
        except Exception as e:
            self._log(f"[gui] worker crashed: {e}\n")
            self._set_status("Crashed")
        finally:
            # tear down everything spawned
            self._kill_all()
            self.root.after(0, lambda: self.btn_sim.configure(state="normal"))
            self.root.after(0, lambda: self.btn_stop.configure(state="disabled"))

    # --------------------------------------------------- process management --
    def _kill_stale_only(self):
        """Kill lingering PX4 / mavsdk_server from a prior run, and scrub
        per-instance rootfs directories. Parameters/dataman left over from a
        previous (different-airframe) boot can poison the next boot —
        especially when switching between none_iris and sihsim_quadx."""
        for pat in ("bin/px4 -i", "mavsdk_server"):
            subprocess.run(["pkill", "-9", "-f", pat],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # PX4's per-instance rootfs is `/tmp/px4-<N>/` (also `rootfs_<N>` in
        # some of our prior tooling). Remove both patterns.
        subprocess.run(["bash", "-c",
                        "rm -rf /tmp/px4-* /tmp/sitl_run/rootfs_* 2>/dev/null || true"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _kill_all(self):
        """Kill controller + mavsdk servers + PX4 launched by this GUI run."""
        for p in (self.proc_controller, self.proc_px4_launcher, *self.proc_mavsdk):
            if p is None:
                continue
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
        time.sleep(0.5)
        # belt & suspenders: anything lingering gets SIGKILL via pkill
        self._kill_stale_only()
        self.proc_controller = None
        self.proc_px4_launcher = None
        self.proc_mavsdk = []

    # --------------------------------------------------- log helpers/async --
    def _log(self, msg: str):
        self.log_queue.put(msg)
        if self._log_fh:
            try:
                self._log_fh.write(msg)
                self._log_fh.flush()
            except Exception:
                pass

    def _set_status(self, s: str):
        self.root.after(0, lambda: self.status.set(s))

    def _pump_stream(self, stream, prefix: str):
        try:
            for line in iter(stream.readline, ""):
                if not line:
                    break
                self._log(prefix + line)
        except Exception:
            pass

    def _drain_log(self):
        try:
            while True:
                msg = self.log_queue.get_nowait()
                self.log.insert("end", msg)
                self.log.see("end")
        except queue.Empty:
            pass
        self.root.after(100, self._drain_log)

    def _interruptible_sleep(self, seconds: float) -> bool:
        """Sleep in 0.2s slices; return True if cancelled."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if self.sim_cancel.is_set():
                return True
            time.sleep(0.2)
        return False

    def _on_close(self):
        if self.sim_thread and self.sim_thread.is_alive():
            if not messagebox.askyesno("Quit",
                                       "A simulation is running. Stop it and quit?"):
                return
            self.sim_cancel.set()
            self._kill_all()
        if self._log_fh:
            try:
                self._log_fh.close()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        style = ttk.Style(root)
        if "clam" in style.theme_names():
            style.theme_use("clam")
    except Exception:
        pass
    SwarmGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
