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
    plt.close(fig)


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
        posi, acce, gyro_00, magn, ahrs = read_log_file(filename)
        velo = do_integration(acce)                               # estimate velocity by integration
        posi = do_integration(velo)                               # estimate position by integration
        acce_nog = do_remove_gravity(acce)                        # remove gravity
        velo_nog = do_integration(acce_nog)                       # estimate velocity by integration
        posi_nog = do_integration(velo_nog)                       # estimate position by integration

        # Ploting
        plot_data3D(acce, "test_" + test + "_acce_row.png", ['AcceX', 'AcceY', 'AcceZ'])
        plot_data3D(ahrs, "test_" + test + "_ahrs_row.png", ['PitchX', 'RollY', 'YawZ'])
        plot_data3D(velo, "test_" + test + "_velo_row.png", ['VeloX', 'VeloY', 'VeloZ'])
        plot_data3D(posi, "test_" + test + "_posi_row.png", ['VeloX', 'VeloY', 'VeloZ'])
        plot_data3D(acce_nog, "test_" + test + "_acce_nog.png", ['AcceX', 'AcceY', 'AcceZ'])
        plot_data3D(velo_nog, "test_" + test + "_velo_nog.png", ['VeloX', 'VeloY', 'VeloZ'])
        plot_data3D(posi_nog, "test_" + test + "_posi_nog.png", ['VeloX', 'VeloY', 'VeloZ'])


    #     posi_00, acce_00, gyro_00, magn_00, ahrs_00 = read_log_file("test_00.log")
    # velo_00 = do_integration(acce_00)             # estimate velocity by integration
    # posi_00 = do_integration(velo_00)             # estimate position by integration
    #
    # acce_nog_00 = do_remove_gravity(acce_00)       # remove gravity
    # velo_nog_00 = do_integration(acce_nog_00)      # estimate velocity by integration
    # posi_nog_00 = do_integration(velo_00)         # estimate position by integration
    #
    # plot_data3D(acce_00, "test_00_acce_row.png",  ['AcceX','AcceY','AcceZ'])
    # plot_data3D(ahrs_00, "test_00_ahrs_row.png",  ['PitchX', 'RollY', 'YawZ'])
    # plot_data3D(velo_00, "test_00_velo_row.png",  ['VeloX', 'VeloY', 'VeloZ'])
    # plot_data3D(posi_00, "test_00_posi_row.png", ['VeloX', 'VeloY', 'VeloZ'])
    #
    # plot_data3D(acce_nog_00, "test_00_acce_nog.png", ['AcceX','AcceY','AcceZ'])
    # plot_data3D(velo_nog_00, "test_00_velo_nog.png",  ['VeloX', 'VeloY', 'VeloZ'])
    # plot_data3D(posi_nog_00, "test_00_posi_row.png", ['VeloX', 'VeloY', 'VeloZ'])
    #
    #
    # posi_15, acce_15, gyro_15, magn_15, ahrs_15 = read_log_file("test_15.log")
    # plot_data3D(acce_15, "test_15_acce_row.png", ['AcceX','AcceY','AcceZ'])
    # plot_data3D(ahrs_15, "test_15_ahrs_row.png",  ['PitchX', 'RollY', 'YawZ'])
    # velo_15 = do_velocity(acce_15)
    # plot_data3D(velo_15, "test_15_velo_row.png",  ['VeloX', 'VeloY', 'VeloZ'])
    #
    # acce_nog_15 = do_remove_gravity(acce_15)
    # plot_data3D(acce_nog_15, "test_15_acce_nog.png", ['AcceX','AcceY','AcceZ'])
    # velo_nog_15 = do_velocity(acce_nog_15)
    # plot_data3D(velo_nog_15, "test_15_velo_nog.png",  ['VeloX', 'VeloY', 'VeloZ'])
    #
    # posi_tr, acce_tr, gyro_tr, magn_tr, ahrs_tr = read_log_file("test_tr.log")
    # acce_tr = remove_first_seconds(acce_tr, 30)
    # gyro_tr = remove_first_seconds(gyro_tr, 30)
    # magn_tr = remove_first_seconds(magn_tr, 30)
    # ahrs_tr = remove_first_seconds(ahrs_tr, 30)
    #
    # plot_data3D(acce_tr, "track_acce_row.png",  ['AcceX','AcceY','AcceZ'])
    # plot_data3D(ahrs_tr, "track_ahrs_row.png",  ['PitchX', 'RollY', 'YawZ'])
    # velo_tr = do_velocity(acce_tr)
    # plot_data3D(velo_tr, "track_velo_row.png",  ['VeloX', 'VeloY', 'VeloZ'])
    #
    # acce_nog_tr = do_remove_gravity(acce_tr)
    # plot_data3D(acce_nog_tr, "track_acce_nog.png", ['AcceX','AcceY','AcceZ'])
    # velo_nog_tr = do_velocity(acce_nog_tr)
    # plot_data3D(velo_nog_tr, "track_velo_nog.png",  ['VeloX', 'VeloY', 'VeloZ'])
    #
    #
