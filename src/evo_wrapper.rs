use pyo3::{prelude::*, types::{IntoPyDict, PyModule}};

pub fn test() -> PyResult<()>{

    Python::with_gil(|py| {
        // let builtins = PyModule::import(py, "builtins")?;
        // let total: i32 = builtins
        //     .getattr("sum")?
        //     .call1((vec![1, 2, 3],))?
        //     .extract()?;

        let numpy = PyModule::import(py, "numpy")?;

        let total2: String = numpy.getattr("__version__")?
                               .extract()?;

        // assert_eq!(total, 6);
        println!("{}", total2);
        Ok(())
    })


    // Python::with_gil(|py| {
    //     let sys = py.import("sys")?;
    //     let version: String = sys.getattr("version")?.extract()?;
    //
    //     let locals = [("os", py.import("os")?)].into_py_dict(py);
    //     let code = "os.getenv('USER') or os.getenv('USERNAME') or 'Unknown'";
    //     let user: String = py.eval(code, None, Some(&locals))?.extract()?;
    //
    //     println!("Hello {}, I'm Python {}", user, version);
    //     Ok(())
    // })

}


// pub fn run_slam_benchmark() -> PyResult<()> {
//     Python::with_gil(|py| {
//         // Importing required modules
//         let evo_core = PyModule::new(py, "evo.core")?;
//         let metrics = PyModule::new(py, "evo.core.metrics")?;
//         let units = PyModule::new(py, "evo.core.units")?;
//         let log = PyModule::new(py, "evo.tools.log")?;
//         let tools = PyModule::new(py, "evo.tools")?;
//         let file_interface = PyModule::new(py, "evo.tools.file_interface")?;
//         let sync = PyModule::new(py, "evo.core.sync")?;
//
//         // Setting up logging
//         log.setattr("configure_logging", (true, true, false))?;
//
//         // Reading trajectory files
//         let ref_file = "../test/data/freiburg1_xyz-groundtruth.txt";
//         let est_file = "../test/data/freiburg1_xyz-rgbdslam_drift.txt";
//
//         let traj_ref: PyObject = file_interface.call1("read_tum_trajectory_file", (ref_file,))?.to_object(py)?;
//         let traj_est: PyObject = file_interface.call1("read_tum_trajectory_file", (est_file,))?.to_object(py)?;
//
//         // Synchronizing trajectories
//         let max_diff = 0.01;
//         let associated_traj = sync.call1("associate_trajectories", (traj_ref, traj_est, max_diff))?;
//
//         // Aligning trajectories
//         let traj_est_aligned: PyObject = associated_traj.getattr("align")((traj_ref, false, false))?;
//
//         // Configuring pose relation and trajectory data
//         let pose_relation: PyObject = metrics.getattr("PoseRelation")?.getattr("translation_part")?;
//         let use_aligned_trajectories = false;
//
//         let data = if use_aligned_trajectories {
//             (traj_ref, traj_est_aligned)
//         } else {
//             (traj_ref, traj_est)
//         };
//
//         // Calculating APE
//         let ape_metric: PyObject = metrics.getattr("APE")?.call1((pose_relation,))?;
//         ape_metric.call1("process_data", (data,))?;
//
//         Ok(())
//     })
// }
//
//
