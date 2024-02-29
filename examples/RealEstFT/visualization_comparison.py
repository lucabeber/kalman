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

    try:
        sp.run(["cmake", "--build", "build/", "--config", "Debug", "--target", "all", "-j", "18"])
    except:
        raise RuntimeError("Failed to build example")
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--makeplot", help="Set to visualize output.", action="store_true")
    parser.add_argument("--path_to_exec", help="Path to executable, relative or absolute", default="../../build")
    args = parser.parse_args()

    # raise a generic RunTimeError if executable fails for any reason
    try:
        lines = run_example(args.path_to_exec + "/experiment")
    except:
        raise RuntimeError("Failed to run experiment")
    
    # compile output into numpy array
    data = None
    for line in lines:
        # convert to string
        line_string = str(line.splitlines()[0], "utf-8")
        line_data = np.array([float(s) for s in line_string.split(",")]).reshape((29, 1))
        if data is None:
            data = line_data
        else:
            data = np.hstack((data, line_data))
    # length of the data
    # plot the data only if flag is set
    end_time = 10
    if args.makeplot:
        # create a numpy array of time steps every 1/500 seconds
        time_steps = np.arange(0, end_time, 1/500)
        length = len(time_steps)
        # Create 4 subplots
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1)
        ax1.plot(time_steps, data[2, :length], color="r", label="x1")
        ax1.plot(time_steps, data[4, :length], color="g", label="x1-ekf")
        ax1.plot(time_steps, data[8, :length], color="b", label="x1-ukf")
        ax1.plot(time_steps, data[16, :length], color="orange", label="x1-sim")
        ax2.plot(time_steps, data[3, :length], color="r", label="x2")
        ax2.plot(time_steps, data[5, :length], color="g", label="x2-ekf")
        ax2.plot(time_steps, data[9, :length], color="b", label="x2-ukf")
        ax2.plot(time_steps, data[17, :length], color="orange", label="x2-sim")
        ax1.legend()
        ax2.legend()
        ax3.plot(time_steps, data[6, :length], color="g", label="x3-ekf")
        ax3.plot(time_steps, data[10, :length], color="b", label="x3-ukf")
        ax3.plot(time_steps, data[18, :length], color="orange", label="x3-sim")
        #ax3.plot(time_steps, data[14, :], color="orange", label="x3-afekf")
        ax4.plot(time_steps, data[7, :length], color="g", label="x4-ekf")
        ax4.plot(time_steps, data[11, :length], color="b", label="x4-ukf")
        ax4.plot(time_steps, data[19, :length], color="orange", label="x4-sim")
        # ax4.plot(time_steps, data[15, :], color="orange", label="x4-afekf")
        ax3.legend()
        ax4.legend()
        # ax2.set_xlabel("Timestep")
        # ax2.set_ylabel("RMS Error")
        # ax2.set_title("RMS Error of Cartesian Position")
        # Set window title
        plt.show()
        fig, (ax5, ax6) = plt.subplots(2, 1)
        # Plot the covariance of the estimate state ekf
        ax5.plot(time_steps, data[20, :length], color="g", label="P11-ekf")
        ax5.plot(time_steps, data[21, :length], color="b", label="P22-ekf")
        ax5.plot(time_steps, data[22, :length], color="orange", label="P33-ekf")
        ax5.plot(time_steps, data[23, :length], color="purple", label="P44-ekf")
        ax5.legend()
        ax6.plot(time_steps, data[24, :length], color="g", label="P11-ukf")
        ax6.plot(time_steps, data[25, :length], color="b", label="P22-ukf")
        ax6.plot(time_steps, data[26, :length], color="orange", label="P33-ukf")
        ax6.plot(time_steps, data[27, :length], color="purple", label="P44-ukf")
        ax6.legend()
        plt.show()

        # Plot the reconstructed force vs the real force
        fig, (ax1, ax2) = plt.subplots(2, 1)
        ax1.plot(time_steps, data[-1, :length], color="r", label="F")
        ax1.plot(time_steps, data[4, :length] * data[6, :length] + data[5, :length] * data[7, :length], color="g", label="F-ekf")
        ax1.plot(time_steps, data[8, :length] * data[10, :length] + data[9, :length] * data[11, :length], color="b", label="F-ukf")
        # error between the estimated and the real force
        ax2.plot(time_steps, data[-1, :length] - data[4, :length] * data[6, :length] - data[5, :length] * data[7, :length], color="r", label="F-err-ekf")
        ax2.plot(time_steps, data[-1, :length] - data[8, :length] * data[10, :length] - data[9, :length] * data[11, :length], color="b", label="F-err-ukf")
        ax1.legend()
        ax2.legend()
        plt.show()


    
    


    # Estimate the mean square error of the estimate penetration data[2, :] and data[4, :], 
    # and of the estimated velocity data[3, :] and data[5, :].
    mean_square_error_pen = np.mean((data[2, :] - data[16, :])**2)/np.size(data[2, :])
    mean_square_error_vel = np.mean((data[3, :] - data[17, :])**2)/np.size(data[3, :])
    

    # Print in terminal the mean square error of the estimate penetration and velocity
    print("Mean square error of the estimate penetration: ", mean_square_error_pen)
    print("Mean square error of the estimate velocity: ", mean_square_error_vel)
    # Print in terminal the stiffness and damping at 2 seconds    
    print("Stiffness at 2 seconds efk: ", data[6, 2*500])
    print("Stiffness at 2 seconds ukf: ", data[10, 2*500])
    print("Damping at 2 seconds efk: ", data[7, 2*500])
    print("Damping at 2 seconds ukf: ", data[11, 2*500])

    # Print in terminal the stiffness and damping at 10 seconds
    print("Stiffness at 10 seconds efk: ", data[6, 10*500])
    print("Stiffness at 10 seconds ukf: ", data[10, 10*500])
    print("Damping at 10 seconds efk: ", data[7, 10*500])
    print("Damping at 10 seconds ukf: ", data[11, 10*500])

        

