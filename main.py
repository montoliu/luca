import bisect
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation


def read_log_file(filename):
    acce = []
    posi = []
    gyro = []
    magn = []
    ahrs = []

    with open(filename, 'r') as file:
        for line in file:
            if line[0] == '%':
                continue
            fields = line.split(';')

            field_type = fields[0]

            if field_type == "POSI":
                posi.append([float(fields[1]), float(fields[3]), float(fields[4])])
            elif field_type == "ACCE":
                acce.append([float(fields[1]), float(fields[3]), float(fields[4]), float(fields[5])])
            elif field_type == "GYRO":
                gyro.append([float(fields[1]), float(fields[3]), float(fields[4]), float(fields[5])])
            elif field_type == "MAGN":
                magn.append([float(fields[1]), float(fields[3]), float(fields[4]), float(fields[5])])
            elif field_type == "AHRS":
                ahrs.append([float(fields[1]), float(fields[3]), float(fields[4]), float(fields[5])])

    return posi, acce, gyro, magn, ahrs


def plot_data3D(data, filename, series_names):
    time = [entry[0] for entry in data]
    x = [entry[1] for entry in data]
    y = [entry[2] for entry in data]
    z = [entry[3] for entry in data]

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharey=True, figsize=(10, 12))

    # Plot x, y, and z in separate subplots
    ax1.plot(time, x, label=series_names[0])
    ax1.set_ylabel(series_names[0])
    ax1.set_title(series_names[0])

    ax2.plot(time, y, label=series_names[1])
    ax2.set_ylabel(series_names[1])
    ax2.set_title(series_names[1])

    ax3.plot(time, z, label=series_names[2])
    ax3.set_xlabel('Time')
    ax3.set_ylabel(series_names[2])
    ax3.set_title(series_names[2])

    ax1.legend()
    ax2.legend()
    ax3.legend()

    plt.savefig(filename)
    plt.close(fig)


def plot_data2D(data, filename, limits):
    x = [entry[1] for entry in data]
    y = [entry[2] for entry in data]
    plt.plot(x, y)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim(limits[0][0], limits[0][1])
    plt.ylim(limits[1][0], limits[1][1])
    plt.savefig(filename)
    plt.close()


def numerical_integration(time, values):
    integral = 0
    for i in range(1, len(time)):
        dt = time[i] - time[i - 1]
        integral += (values[i] + values[i - 1]) / 2 * dt
    return integral


def do_integration(data_in):
    time = [float(entry[0]) for entry in data_in]
    x = [entry[1] for entry in data_in]
    y = [entry[2] for entry in data_in]
    z = [entry[3] for entry in data_in]
    data_out = []
    data_out.append([time[0], 0.0, 0.0, 0.0])

    for i in range(1, len(time)):
        dt = time[i] - time[i - 1]
        velo_x = data_out[i - 1][1] + (x[i] + x[i - 1]) / 2 * dt
        velo_y = data_out[i - 1][2] + (y[i] + y[i - 1]) / 2 * dt
        velo_z = data_out[i - 1][3] + (z[i] + z[i - 1]) / 2 * dt

        data_out.append([time[i], velo_x, velo_y, velo_z])

    return data_out


def do_remove_gravity_by_mean(data):
    time = [entry[0] for entry in data]
    x = [entry[1] for entry in data]
    y = [entry[2] for entry in data]
    z = [entry[3] for entry in data]

    mean_x = np.mean(x)
    mean_y = np.mean(y)
    mean_z = np.mean(z)

    data_nog = []
    for i in range(len(time)):
        data_nog.append([time[i], x[i] - mean_x, y[i] - mean_y, z[i] - mean_z])

    return data_nog


def look_time_most_similar(time_list, time):
    index = bisect.bisect_left(time_list, time)
    if index > 0 and (index == len(time_list) or abs(time - time_list[index - 1]) < abs(time_list[index] - time)):
        return index - 1
    return index


def do_remove_gravity_by_rotation(acce, ahrs):
    time_acce = [entry[0] for entry in acce]
    acce_x = [entry[1] for entry in acce]
    acce_y = [entry[2] for entry in acce]
    acce_z = [entry[3] for entry in acce]

    time_ahrs = [entry[0] for entry in ahrs]
    pitch_x = [entry[1] for entry in ahrs]
    roll_y = [entry[2] for entry in ahrs]
    yaw_z = [entry[3] for entry in ahrs]

    acce_nog = []
    for i in range(len(time_acce)):
        idx = look_time_most_similar(time_ahrs, time_acce[i])
        alpha = to_rad(yaw_z[idx])  # YawZ
        beta = to_rad(roll_y[idx])  # RollY
        gamma = to_rad(pitch_x[idx])  # PitchX

        acce_ith = np.array([acce_x[i], acce_y[i], acce_z[i]])

        ca = np.cos(alpha)
        sa = np.sin(alpha)
        cb = np.cos(beta)
        sb = np.sin(beta)
        cg = np.cos(gamma)
        sg = np.sin(gamma)

        Ralhpa = np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]])
        Rbeta = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
        Rgamma = np.array([[1, 0, 0], [0, cg, -sg], [0, sg, cg]])

        Rtemp = np.dot(Ralhpa, Rbeta)
        R = np.dot(Rtemp, Rgamma)

        new_acce = np.dot(R, acce_ith) - np.array([0, 0, 9.8])
        acce_nog.append([time_acce[i], new_acce[0], new_acce[1], new_acce[2]])

    return acce_nog


def to_rad(deg):
    return deg * np.pi / 180.0


def remove_first_seconds(data, seconds):
    time = [float(entry[0]) for entry in data]
    # look point
    i = 0
    while time[i] < seconds:
        i += 1

    return data[i:]


if __name__ == '__main__':

    l_tests = ['00', '15', 'tr']

    for test in l_tests:
        filename = "test_" + test + ".log"
        print("Working with file: " + filename)
        posi_org, acce, gyro, magn, ahrs = read_log_file(filename)
        if test == 'tr':
            acce = remove_first_seconds(acce, 30)
            ahrs = remove_first_seconds(ahrs, 30)
            posi_org = remove_first_seconds(posi_org, 30)

        velo_row = do_integration(acce)                           # estimate velocity by integration
        posi_row = do_integration(velo_row)                       # estimate position by integration

        # acce_nog = do_remove_gravity_by_mean(acce)                # remove gravity by mean
        acce_nog = do_remove_gravity_by_rotation(acce, ahrs)      # remove gravity by mean

        velo_nog = do_integration(acce_nog)                       # estimate velocity by integration
        posi_nog = do_integration(velo_nog)                       # estimate position by integration

        # Ploting
        plot_data3D(acce, "test_" + test + "_acce_row.png", ['AcceX', 'AcceY', 'AcceZ'])
        plot_data3D(ahrs, "test_" + test + "_ahrs_row.png", ['PitchX', 'RollY', 'YawZ'])
        plot_data3D(velo_row, "test_" + test + "_velo_row.png", ['VeloX', 'VeloY', 'VeloZ'])
        plot_data3D(posi_row, "test_" + test + "_posi_row.png", ['VeloX', 'VeloY', 'VeloZ'])
        plot_data3D(acce_nog, "test_" + test + "_acce_nog.png", ['AcceX', 'AcceY', 'AcceZ'])
        plot_data3D(velo_nog, "test_" + test + "_velo_nog.png", ['VeloX', 'VeloY', 'VeloZ'])
        plot_data3D(posi_nog, "test_" + test + "_posi_nog.png", ['VeloX', 'VeloY', 'VeloZ'])

        if test == 'tr':
            limits_org = [[43.7182, 43.7186], [10.4215, 10.4223]]
            limits_raw = [[-10000, 10000], [-10000, 10000]]
            limits_nog = [[-10000, 10000], [-1000, 10000]]
        else:
            limits_org = [[0, 10], [0, 10]]
            limits_raw = [[-250, 250], [-250, 250]]
            limits_nog = [[-100, 100], [-100, 100]]

        plot_data2D(posi_org, "test_" + test + "_posi2D_org.png", limits_org)
        plot_data2D(posi_row, "test_" + test + "_posi2D_row.png", limits_raw)
        plot_data2D(posi_nog, "test_" + test + "_posi2D_nog.png", limits_nog)



