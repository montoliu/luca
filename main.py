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


def plot_data(data, filename, series_names):
    time = [float(entry[0]) for entry in data]
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


def numerical_integration(time, values):
    integral = 0
    for i in range(1, len(time)):
        dt = time[i] - time[i - 1]
        integral += (values[i] + values[i - 1]) / 2 * dt
    return integral


def do_velocity(data):
    time = [float(entry[0]) for entry in data]
    x = [entry[1] for entry in data]
    y = [entry[2] for entry in data]
    z = [entry[3] for entry in data]
    velo = []
    velo.append([time[0], 0.0, 0.0, 0.0])

    for i in range(1, len(time)):
        dt = time[i] - time[i - 1]
        velo_x = velo[i - 1][1] + (x[i] + x[i - 1]) / 2 * dt
        velo_y = velo[i - 1][2] + (y[i] + y[i - 1]) / 2 * dt
        velo_z = velo[i - 1][3] + (z[i] + x[i - 1]) / 2 * dt

        velo.append([time[i], velo_x, velo_y, velo_z])

    return velo


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
    posi, acce, gyro, magn, ahrs = read_log_file("test_00.log")
    plot_data(acce, "test_00_acce_row.png",  ['AcceX','AcceY','AcceZ'])
    plot_data(ahrs, "test_00_ahrs_row.png",  ['PitchX', 'RollY', 'YawZ'])
    velo = do_velocity(acce)
    plot_data(velo, "test_00_velo_row.png",  ['VeloX', 'VeloY', 'VeloZ'])

    acce_nog = do_remove_gravity(acce)
    plot_data(acce_nog, "test_00_acce_nog.png", ['AcceX','AcceY','AcceZ'])
    velo_nog = do_velocity(acce_nog)
    plot_data(velo_nog, "test_00_velo_nog.png",  ['VeloX', 'VeloY', 'VeloZ'])

    posi, acce, gyro, magn, ahrs = read_log_file("test_15.log")
    plot_data(acce, "test_15_acce_row.png", ['AcceX','AcceY','AcceZ'])
    plot_data(ahrs, "test_15_ahrs_row.png",  ['PitchX', 'RollY', 'YawZ'])
    velo = do_velocity(acce)
    plot_data(velo, "test_15_velo_row.png",  ['VeloX', 'VeloY', 'VeloZ'])

    acce_nog = do_remove_gravity(acce)
    plot_data(acce_nog, "test_15_acce_nog.png", ['AcceX','AcceY','AcceZ'])
    velo_nog = do_velocity(acce_nog)
    plot_data(velo_nog, "test_15_velo_nog.png",  ['VeloX', 'VeloY', 'VeloZ'])

    posi, acce, gyro, magn, ahrs = read_log_file("track.log")
    acce = remove_first_seconds(acce, 30)
    gyro = remove_first_seconds(gyro, 30)
    magn = remove_first_seconds(magn, 30)
    ahrs = remove_first_seconds(ahrs, 30)

    plot_data(acce, "track_acce_row.png",  ['AcceX','AcceY','AcceZ'])
    plot_data(ahrs, "track_ahrs_row.png",  ['PitchX', 'RollY', 'YawZ'])
    velo = do_velocity(acce)
    plot_data(velo, "track_velo_row.png",  ['VeloX', 'VeloY', 'VeloZ'])

    acce_nog = do_remove_gravity(acce)
    plot_data(acce_nog, "track_acce_nog.png", ['AcceX','AcceY','AcceZ'])
    velo_nog = do_velocity(acce_nog)
    plot_data(velo_nog, "track_velo_nog.png",  ['VeloX', 'VeloY', 'VeloZ'])


