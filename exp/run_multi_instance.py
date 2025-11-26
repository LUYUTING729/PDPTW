import argparse
import json
import os
import subprocess
import time
from typing import List, Tuple, Optional
from pathlib import Path

def find_initial_solution_files(instance_name: str, bks_dir: str) -> List[str]:
    """
    查找给定实例的初始解文件
    :param instance_name: 实例名称（如 LC1_4_3）
    :param bks_dir: BKS文件夹路径
    :return: 找到的初始解文件列表（完整路径）
    """
    result = []
    instance_name_lower = instance_name.lower()
    
    if not os.path.isdir(bks_dir):
        return result
    
    # 搜索以instance_name开头的文件（不区分大小写）
    for file in os.listdir(bks_dir):
        if file.lower().startswith(instance_name_lower):
            full_path = os.path.join(bks_dir, file)
            if os.path.isfile(full_path):
                result.append(full_path)
    
    return sorted(result)  # 排序保证结果一致性

def run_command_with_monitor(
    cmd: List[str],
    log_file: str,
    time_limit: int = 7500,
) -> Tuple[bool, int, str]:
    """
    执行命令并监控:创建子进程,异常不会中断程序，按指定路径保存日志
    :param cmd: 执行命令列表
    :param log_file: 单个算例的独立日志文件路径
    :param time_limit: 超时时间（秒）
    :return: (执行成功标志, 进程返回码, 状态)
    """
    proc: Optional[subprocess.Popen] = None
    success = False
    return_code = 1
    status = "OK"

    log_dir = os.path.dirname(log_file)
    os.makedirs(log_dir, exist_ok=True)

    try:
        with open(log_file, "w", encoding="utf-8", errors="ignore") as lf:
            start_time = time.time()
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="ignore",
                preexec_fn=os.setsid
            )
            lf.write(f"[AMDAHL] PID: {proc.pid}\n")
            lf.write(f"[AMDAHL] Command: {' '.join(cmd)}\n")
            lf.write(f"[AMDAHL] Time Limit: {time_limit}s\n")
            lf.write("=" * 60 + "\n")
            lf.flush()

            assert proc.stdout is not None

            # 实时读取输出，带超时检查
            while True:
                elapsed = time.time() - start_time
                if elapsed > time_limit:
                    status = "TIMEOUT"
                    lf.write(f"\n[AMDAHL] Timeout ({time_limit}s), killing process\n")
                    lf.flush()
                    if proc and proc.poll() is None:
                        try:
                            os.killpg(proc.pid, 9)
                            proc.wait(timeout=5)
                        except Exception:
                            pass
                    return_code = 124
                    break

                line = proc.stdout.readline()
                if line:
                    lf.write(line)
                    lf.flush()

                if proc.poll() is not None:
                    return_code = proc.returncode
                    success = return_code == 0
                    status = "OK" if success else f"ERROR({return_code})"
                    break

                time.sleep(0.1)

            remaining = proc.stdout.read()
            if remaining:
                lf.write(remaining)
                lf.flush()

    except FileNotFoundError as e:
        status = "FILE_NOT_FOUND"
        with open(log_file, "w", encoding="utf-8") as lf:
            lf.write(f"[Error] {e}\n")
    except Exception as e:
        status = "EXCEPTION"
        with open(log_file, "a", encoding="utf-8") as lf:
            lf.write(f"[Exception] {e}\n")
    finally:
        if proc and proc.poll() is None:
            try:
                os.killpg(proc.pid, 9)
            except Exception:
                pass

    return success, return_code, status

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Batch run AMDAHL solver")
    parser.add_argument("--config", type=str, default="/home/yutinglu/amdahl/src/config.json")
    parser.add_argument("--binary", type=str, default="/home/yutinglu/amdahl/src/build/amdahl")
    parser.add_argument("--result-dir", type=str, default="/home/yutinglu/amdahl/src/result")
    parser.add_argument("--time-limit", type=int, default=7200)
    
    args = parser.parse_args()

    # Validate config and binary
    if not os.path.isfile(args.config):
        print(f"[Error] Config not found: {args.config}")
        exit(1)
    
    if not os.path.isfile(args.binary) or not os.access(args.binary, os.X_OK):
        print(f"[Error] Binary not found or not executable: {args.binary}")
        exit(1)

    os.makedirs(args.result_dir, exist_ok=True)

    # Load base config
    with open(args.config, 'r', encoding='utf-8') as f:
        base_config = json.load(f)

    # Define instances to process
    instances = [
        "LC1_4_3", "LC1_10_2", "LC2_10_3",
        "LR1_8_5", "LR1_10_8", "LR2_10_6",
        "LRC1_10_7", "LRC1_10_9", "LRC2_10_3",
        "LRC2_10_10"
    ]

    print("=== Batch Solver ===")
    print(f"Config: {args.config}")
    print(f"Binary: {args.binary}")
    print(f"Result: {args.result_dir}")
    print(f"Instances: {len(instances)}")
    print()

    success_count = 0
    bks_dir = "/home/yutinglu/amdahl/src/data/bks"  # BKS文件夹路径

    # Process each instance
    for idx, instance_name in enumerate(instances, start=1):
        print(f"[{idx}/{len(instances)}] Processing {instance_name}...")

        # Generate config for this instance
        temp_cfg = os.path.join(args.result_dir, f"cfg_{instance_name}.json")
        cfg = base_config.copy()
        cfg['instanceName'] = instance_name
        cfg['timeLimit'] = args.time_limit
        
        # 查找并配置初始解文件
        initial_sol_files = find_initial_solution_files(instance_name, bks_dir)
        cfg['initialSolFiles'] = initial_sol_files
        
        cfg['improvedVersion'] = False

        with open(temp_cfg, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)

        log_file = os.path.join(args.result_dir, f"{instance_name}.log")
        cmd = [args.binary, temp_cfg]
        
        success, rc, status = run_command_with_monitor(
            cmd=cmd,
            log_file=log_file,
            time_limit=7500,
        )

        if success:
            success_count += 1
            print(f"  ✓ {instance_name}: OK")
        else:
            print(f"  ✗ {instance_name}: {status} (rc={rc})")

    print()
    print("=" * 60)
    print(f"Completed: {success_count}/{len(instances)}")
    print(f"Output: {args.result_dir}")
