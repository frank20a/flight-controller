def plot_imu(data, axes, lims = [None, None, None], title_apnds=['', '', '']):
    ax1, ax2, ax3 = axes
    
    ax1.plot(data['ax'], label='Accel X')
    ax1.plot(data['ay'], label='Accel Y')
    ax1.plot(data['az'], label='Accel Z')
    ax1.legend()
    ax1.set_title(f'Accelerometer {title_apnds[0]}')
    ax1.set_ylabel('Acceleration (m/s^2)')
    if lims[0] != None:
        ax1.set_ylim(lims[0])

    ax2.plot(data['mx'], label='Mag X')
    ax2.plot(data['my'], label='Mag Y')
    ax2.plot(data['mz'], label='Mag Z')
    ax2.legend()
    ax2.set_title(f'Magnetometer {title_apnds[1]}')
    ax2.set_ylabel('Magnetic Field (uT)')
    if lims[1] != None:
        ax2.set_ylim(lims[1])
    

    ax3.plot(data['gx'], label='Gyro X')
    ax3.plot(data['gy'], label='Gyro Y')
    ax3.plot(data['gz'], label='Gyro Z')
    ax3.legend()
    ax3.set_title(f'Gyroscope {title_apnds[2]}')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angular Velocity (rad/s)')
    if lims[2] != None:
        ax3.set_ylim(lims[2])