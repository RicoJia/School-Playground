
import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Config:
    mode: str                 # "talker" | "listener"
    num_topics: int           # number of publishers/subscribers
    num_nodes: int            # number of nodes
    msg_kb: str               # message schema size in KB (e.g., "128B", "1KB", "15KB")
    transport_variant: str    # transport variant: "UDP", "SHM", "SHM-zero-copy"
    duration_s: float = 45.0   # talker: active publishing time before stop flag
    poll_ms: int = 200        # listener: stop-flag poll interval
    stop_dir: Path = Path("/tmp/multi-container-shm-test")  # typically /tmp
    logs_dir: Path = Path("/tmp/multi-container-shm-test")  # typically /tmp (can be /tmp/<run_id>/)
    best_effort: bool = False  # placeholder for QoS mode
    create_dirs: bool = False  # create run log dir if missing
    runId: str = ""           # run ID for logging directory


def parse_args(argv: list[str]) -> Config:
    p = argparse.ArgumentParser(description="Two-container perf test prototype (prints only).")
    p.add_argument("--mode", choices=["talker", "listener"], required=True)
    p.add_argument("--num-topics", "-n", type=int, required=True, help="Number of pubs/subs.")
    p.add_argument("--num-nodes", type=int, required=True, help="Number of nodes.")
    p.add_argument("--msg-kb", required=True, type=str, help="Message size (e.g., 128B, 1KB, 15KB).")
    p.add_argument("--transport-variant", choices=["UDP", "SHM", "SHM-zero-copy"], required=True, help="Transport variant.")
    p.add_argument("--run-id", type=str, default=time.strftime("%Y-%m-%d-%H-%M"), help="Run ID for logging directory.")

    a = p.parse_args(argv)

    # Basic sanity checks
    if a.num_topics <= 0:
        print("ERROR: --num-topics must be > 0", file=sys.stderr)
        sys.exit(2)

    if a.num_nodes <= 0:
        print("ERROR: --num-nodes must be > 0", file=sys.stderr)
        sys.exit(2)

    if a.msg_kb.lower() not in ["128b", "1kb", "15kb"]:
        print("ERROR: --msg-kb must be one of: 128B, 1KB, 15KB", file=sys.stderr)
        sys.exit(2)

    return Config(
        mode=a.mode,
        num_topics=a.num_topics,
        num_nodes=a.num_nodes,
        msg_kb=a.msg_kb,
        transport_variant=a.transport_variant,
        runId=a.run_id,
    )


def event_flag_path(cfg: Config, role: str, node_id: int) -> Path:
    return cfg.stop_dir / f"{role}_flag_{node_id}"


def node_log_path(cfg: Config, role: str, node_id: int, run_dir: Path) -> Path:
    return run_dir / f"{role}_nt{cfg.num_topics}_{cfg.msg_kb}_node{node_id}.log"


def ensure_dirs(cfg: Config) -> None:
    if cfg.create_dirs:
        cfg.logs_dir.mkdir(parents=True, exist_ok=True)
        cfg.stop_dir.mkdir(parents=True, exist_ok=True)


def append_line(path: Path, line: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(line.rstrip("\n") + "\n")


def _spawn(cfg: Config, role: str, node_id: int = 0, log_path: Path = None) -> subprocess.Popen:
    """
    Spawn a ROS2 node for talker or listener with the given config.

    Args:
        cfg: Configuration object
        role: "talker" or "listener"
        node_id: Node identifier (default 0)
        log_path: Path to log file for stdout and stderr

    Returns:
        subprocess.Popen: The spawned process
    """
    # Setup environment
    env = os.environ.copy()
    
    # Parse transport_variant
    if cfg.transport_variant == "UDP":
        transport = "UDP"
        zero_copy = False
    elif cfg.transport_variant == "SHM":
        transport = "SHM"
        zero_copy = False
    elif cfg.transport_variant == "SHM-zero-copy":
        transport = "SHM"
        zero_copy = True
    
    if transport in ("SHM", "SHM-zero-copy"):
        env['CYCLONEDDS_URI'] = 'file:///root/toolkitt_ws/src/shm_tests/src/shm_tests/configs/cyclonedds_shm.xml'

    zero_copy_arg = str(zero_copy).lower()

    cmd = (
        "cd /root/toolkitt_ws && "
        "source install/setup.bash && "
        f"ros2 run shm_tests shm_test_many_topics_{role} --ros-args "

        f"--param num_topics:={cfg.num_topics} "
        f"--param message_size:={cfg.msg_kb} "
        f"--param zero_copy:={zero_copy_arg} "
        f"--param node_num:={node_id} "
        "|| echo '[ERROR] ros2 command failed with exit code '$?"
    )

    print(f"[{role.upper()}] Launching node {node_id}: num_topics={cfg.num_topics}, message_size={cfg.msg_kb}, transport_variant={cfg.transport_variant}")
    log_file = open(log_path, 'w', buffering=1)  # Line buffering
    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        stdout=log_file,
        stderr=subprocess.STDOUT,  # Merge stderr into stdout
        env=env,
        start_new_session=True,
    )
    
    # Give process a moment to start and check if it crashes immediately
    time.sleep(0.1)
    if proc.poll() is not None:
        log_file.close()
        print(f"[{role.upper()}] WARNING: Node {node_id} process exited immediately with code {proc.returncode}")
        # Try to read any error output
        with open(log_path, 'r') as f:
            error_output = f.read()
            if error_output:
                print(f"[{role.upper()}] Error output from node {node_id}:")
                print(error_output)
    
    return proc


def _kill_process(proc: subprocess.Popen, role: str, log: Path = None) -> None:
    """
    Kill a single process gracefully.

    Args:
        proc: The subprocess to kill
        role: "talker" or "listener" for logging
        log: Optional log file to append warnings to
    """
    try:
        # Check if process is still running
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5.0)
            print(f"[{role.upper()}] Terminated {role} process")
        else:
            print(f"[{role.upper()}] Process already exited with code {proc.returncode}")
    except (ProcessLookupError, subprocess.TimeoutExpired) as e:
        print(f"[{role.upper()}] Warning: could not terminate {role} process: {e}")
        if log:
            append_line(log, f"[{role.upper()}] Warning: could not terminate {role} process: {e}")
