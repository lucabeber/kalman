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
        lines = run_example(args.path_to_exec + "/example_measurament")
    except:
        raise RuntimeError("Failed to run example_measurement")
    
    # compile output into numpy array
    data = None
    for line in lines:
        # convert to string
        line_string = str(line.splitlines()[0], "utf-8")
        line_data = np.array([float(s) for s in line_string.split(",")]).reshape((8, 1))
        if data is None:
            data = line_data
        else:
            data = np.hstack((data, line_data))

    # plot the data only if flag is set
    if args.makeplot:
        # create a numpy array of time steps every 1/500 seconds
        time_steps = np.arange(0, 10, 1/500)
        # Create 4 subplots
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)
        ax1.plot(time_steps, data[2, :], color="r", label="x1")
        ax1.plot(time_steps, data[4, :], color="g", label="x1-ekf")
        ax2.plot(time_steps, data[3, :], color="b", label="x2")
        ax2.plot(time_steps, data[5, :], color="k", label="x2-ekf")
        ax1.legend()
        ax2.legend()
        ax3.plot(time_steps, data[6, :], color="g", label="x3-ekf")
        ax4.plot(time_steps, data[7, :], label="x4-ekf")
        ax3.legend()
        ax4.legend()
        # ax2.set_xlabel("Timestep")
        # ax2.set_ylabel("RMS Error")
        # ax2.set_title("RMS Error of Cartesian Position")
        # Set window title

        plt.show()

    # Estimate the mean square error of the estimate penetration data[2, :] and data[4, :], 
    # and of the estimated velocity data[3, :] and data[5, :].
    mean_square_error_pen = np.mean((data[2, :] - data[4, :])**2)
    mean_square_error_vel = np.mean((data[3, :] - data[5, :])**2)

    # Print in terminal the mean square error of the estimate penetration and velocity
    print("Mean square error of the estimate penetration: ", mean_square_error_pen)
    print("Mean square error of the estimate velocity: ", mean_square_error_vel)
    
        

