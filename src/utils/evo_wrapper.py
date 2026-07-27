from pathlib import Path
import subprocess

def compute_ape(
        bag_path: Path,
        gt_topic: str,
        odom_topic: str,
        output_dir: Path,
    ):
    """Computes Absolute Pose Error using evo."""
    output_dir.mkdir(parents=True, exist_ok=True)

    zip_path = output_dir / "ape_results.zip"
    plot_path = output_dir / "ape_plot.png"

    # Base evo_ape command for ROS 2 bags
    cmd = ["evo_ape", "bag2", "-a"]

    # Handle ground truth and odometry in same bag vs separate bags
    cmd.extend([str(bag_path), gt_topic, odom_topic])

    # Export result files
    cmd.extend(["--save_results", str(zip_path)])

    # Disable plotting if no display is available
    import os
    if 'DISPLAY' not in os.environ:
        print("No display found. Disabling plotting.")
        cmd.remove("--plot_mode")
        cmd.remove("xyz")
        cmd.remove("--save_plot")
        cmd.append("--no_plot")

    try:
        subprocess.run(
            cmd,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(
            f"evo_ape failed with return code {e.returncode}:\n"
            f"STDOUT:\n{e.stdout}\n"
            f"STDERR:\n{e.stderr}"
        ) from e
