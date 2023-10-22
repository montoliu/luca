import matplotlib.pyplot as plt
import numpy as np


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
                posi.append([float(fields[1]), float(fields[2]), float(fields[3]), float(fields[4])])
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
    plt.xlim(limits[0], limits[1])
    plt.ylim(limits[0], limits[1])
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
        velo_z = data_out[i - 1][3] + (z[i] + x[i - 1]) / 2 * dt

        data_out.append([time[i], velo_x, velo_y, velo_z])

    return data_out


def do_remove_gravity(data):
    time = [float(entry[0]) for entry in data]
    x = [entry[1] for entry in data]
    y = [entry[2] for entry in data]
    z = [entry[3] for entry in data]

    mean_x = np.mean(x)
    mean_y = np.mean(y)
    mean_z = np.mean(z)

    acce_nog = []
    for i in range(len(time)):
        acce_nog.append([time[i], x[i] - mean_x, y[i]-mean_y, z[i]- mean_z])

    return acce_nog


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
        velo_row = do_integration(acce)                           # estimate velocity by integration
        posi_row = do_integration(velo_row)                       # estimate position by integration
        acce_nog = do_remove_gravity(acce)                        # remove gravity
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

        plot_data2D(posi_org, "test_" + test + "_posi2D_org.png", [-10, 10])
        plot_data2D(posi_row, "test_" + test + "_posi2D_row.png", [-100, 100])
        plot_data2D(posi_nog, "test_" + test + "_posi2D_nog.png", [-100, 100])



