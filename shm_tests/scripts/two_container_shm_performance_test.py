#!/usr/bin/env python3

# ros2 run shm_tests two_container_shm_performance_test.py --mode listener  --msg-kb 1KB --transport UDP --zero-copy false --num-nodes 3 --num-topics 1

from shm_performance_test_utils import Config, parse_args, ensure_dirs, node_log_path, event_flag_path, append_line, _spawn, _kill_process
import sys
import os
import time
sys.path.insert(0, os.path.dirname(__file__))

############################################################################################################################################################
# Talker
############################################################################################################################################################

def run_talkers(cfg: Config):
    ensure_dirs(cfg)    # to ensure the flag path and logs have their dirs
    run_dir = cfg.logs_dir / cfg.runId
    run_dir.mkdir(parents=True, exist_ok=True)

    print(f"[TALKER] num_topics={cfg.num_topics} msg_kb={cfg.msg_kb} duration_s={cfg.duration_s} transport_variant={cfg.transport_variant}")

    # spawn_all_talkers
    procs = []
    for node_id in range(cfg.num_nodes):
        node_log = node_log_path(cfg, "talker", node_id, run_dir)
        proc = _spawn(cfg, "talker", node_id, node_log)
        procs.append(proc)

    # Wait for start flags and print started
    for node_id in range(cfg.num_nodes):
        flag = event_flag_path(cfg, "start", node_id)
        while not flag.exists():
            time.sleep(0.01)
        flag.unlink(missing_ok=True)
        print(f"node {node_id} has started publishing")

    # sleep for the test duration
    print(f"[TALKER] Running test for {cfg.duration_s}s...")
    time.sleep(cfg.duration_s)

    # write stop flags for each talker
    for node_id in range(cfg.num_nodes):
        flag = event_flag_path(cfg, "stop", node_id)
        flag.touch()

    # Check for early crashes before we terminate
    crashed_nodes = []
    for i, proc in enumerate(procs):
        if proc.poll() is not None:  # Process already exited
            crashed_nodes.append(i)
    
    if crashed_nodes:
        print(f"[TALKER] WARNING: {len(crashed_nodes)} nodes crashed early: {crashed_nodes}")
    
    # terminate all talkers
    for proc in procs:
        _kill_process(proc, "talker")
    
    print(f"[TALKER] Terminated all {cfg.num_nodes} nodes")

############################################################################################################################################################
# Listener
############################################################################################################################################################


def run_listeners(cfg: Config):
    ensure_dirs(cfg)
    run_dir = cfg.logs_dir / cfg.runId
    run_dir.mkdir(parents=True, exist_ok=True)

    print(f"[LISTENER] num_topics={cfg.num_topics} msg_kb={cfg.msg_kb} transport_variant={cfg.transport_variant}")

    # Spawn all listeners
    procs_and_ids = []
    for node_id in range(cfg.num_nodes):
        node_log = node_log_path(cfg, "listener", node_id, run_dir)
        proc = _spawn(cfg, "listener", node_id, node_log)
        procs_and_ids.append((proc, node_id, node_log))

    # Create start flags for listeners
    for node_id in range(cfg.num_nodes):
        flag = event_flag_path(cfg, "start", node_id)
        flag.touch()

    # Watch for stop flags and terminate corresponding listeners
    while procs_and_ids:
        for proc, node_id, node_log in list(procs_and_ids):
            flag = event_flag_path(cfg, "stop", node_id)
            if flag.exists():
                _kill_process(proc, "listener", node_log)
                try:
                    flag.unlink(missing_ok=True)
                    print(f"[LISTENER] Terminated listener process and removed stop flag: {flag}")
                except OSError as e:
                    print(f"[LISTENER] WARNING could not remove stop flag {flag}: {e}", file=sys.stderr)
                procs_and_ids.remove((proc, node_id, node_log))
        time.sleep(cfg.poll_ms / 1000.0)

    print(f"[LISTENER] Logs saved to {run_dir}")


def main(argv: list[str]) -> int:
    cfg = parse_args(argv)

    if cfg.mode == "talker":
        return run_talkers(cfg)
    return run_listeners(cfg)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
