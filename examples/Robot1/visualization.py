import matplotlib.pyplot as plt
import numpy as np
import subprocess as sp
import argparse

def run_example(_exec):
    """
    run_example
        Args:
        _exec - string, example executable (including relative path)
        
        Returns:
        bytestring, stdout from running provided example
    """
    p = sp.Popen(_exec, stdout=sp.PIPE, stderr=sp.STDOUT)
    return p.stdout.readlines()


if __name__ == "__main__":
    """
    run example_robot1 and, if optional command line flag \"makeplot\" is set, plot the cartesian position
    and it's error for the three filter strategies used in the example: Predict, EKF, UKF
    """

    parser = argparse.ArgumentParser()
    parser.add_argument("--makeplot", help="Set to visualize output.", action="store_true")
    parser.add_argument("--path_to_exec", help="Path to executable, relative or absolute", default="../../build")
    args = parser.parse_args()

    # raise a generic RunTimeError if executable fails for any reason
    try:
        lines = run_example(args.path_to_exec + "/example_robot1")
    except:
        raise RuntimeError("Failed to run example_robot1")
    
    # compile output into numpy array
    data = None
    for line in lines:
        # convert to string
        line_string = str(line.splitlines()[0], "utf-8")
        line_data = np.array([float(s) for s in line_string.split(",")]).reshape((12, 1))
        if data is None:
            data = line_data
        else:
            data = np.hstack((data, line_data))

    # plot the data only if flag is set
    if args.makeplot:
        errors = np.empty((3, data.shape[1]))
        errors[0, :] = np.linalg.norm(data[:2, :] - data[3:5, :], axis=0)
        errors[1, :] = np.linalg.norm(data[:2, :] - data[6:8, :], axis=0)
        errors[2, :] = np.linalg.norm(data[:2, :] - data[9:11, :], axis=0)
        f, (ax1, ax2) = plt.subplots(1, 2)
        ax1.plot(data[0, :], data[1, :], color="r", label="ground-truth")
        ax1.plot(data[3, :], data[4, :], color="g", label="predict-only")
        ax1.plot(data[6, :], data[7, :], color="b", label="ekf")
        ax1.plot(data[9, :], data[10, :], color="k", label="ukf")
        ax1.legend()
        ax1.set_xlabel("x")
        ax1.set_ylabel("y")
        ax1.set_title("Cartesian Position")
        ax2.plot(errors[0, :], color="g")
        ax2.plot(errors[1, :], color="b")
        ax2.plot(errors[2, :], color="k")
        ax2.set_xlabel("Timestep")
        ax2.set_ylabel("RMS Error")
        ax2.set_title("RMS Error of Cartesian Position")
        plt.gcf().canvas.set_window_title("Comparison of Different Filters")
        plt.show()
