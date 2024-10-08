import os
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt, find_peaks  
import matplotlib.pyplot as plt  
from scipy.interpolate import interp1d
import argparse


font_size = 10


def compute_gait_average_profile(input_data):
    # Unpack parameters
    gait_head_list = input_data['gaitHeadList']
    gait_tail_list = input_data['gaitTailList']
    data_seq = input_data['dataSeq']
    data_out_names = input_data['dataOutNames']
    data_shift = input_data['dataShift']
    normalized_gait_length = input_data['normalizedGaitLength']
    
    # Assertions
    assert len(data_seq) == len(data_out_names), 'Error: dataSeq and dataOutNames need to have the same length!'

    # Initialize output
    result = {}

    # Compute
    for idx in range(len(gait_head_list)):
        gait_head = gait_head_list[idx]
        gait_tail = gait_tail_list[idx]
        
        for data_idx in range(len(data_seq)):
            data = data_seq[data_idx]
            data_out_name = data_out_names[data_idx]
            gait_data = data[gait_head:gait_tail]
            
            # Normalize
            x = np.arange(1, len(gait_data) + 1)
            interp_func = interp1d(x, gait_data, kind='linear', fill_value='extrapolate')
            gait_data_normalized = interp_func(np.linspace(1, len(gait_data), normalized_gait_length))

            # Compensate for the added delay
            gait_data_normalized = np.roll(gait_data_normalized, int(round(data_shift)))

            # Gather result
            if f'gait{data_out_name}NormalizedList' not in result:
                result[f'gait{data_out_name}NormalizedList'] = gait_data_normalized[np.newaxis, :]
            else:
                result[f'gait{data_out_name}NormalizedList'] = np.vstack(
                    [result[f'gait{data_out_name}NormalizedList'], gait_data_normalized[np.newaxis, :]]
                )
    
    for data_idx in range(len(data_seq)):
        data_out_name = data_out_names[data_idx]
        result[f'gait{data_out_name}NormalizedAvgList'] = np.mean(
            result[f'gait{data_out_name}NormalizedList'], axis=0
        )
        result[f'gait{data_out_name}NormalizedStdList'] = np.std(
            result[f'gait{data_out_name}NormalizedList'], axis=0
        )

    return result

    
def process_data(
    rootdir = "./data/Controller_100Hz_Gain_0x20_20240905_Yuming", 
    dt_controller = 1/100, 
    dt_logging = 1/100, 
    torque_gain = 0.20/20, 
    right_leg_sign = -1, 
    all_sign_inverse = 1  
):  
    # Get list of files in any subfolder
    filelist = [os.path.join(dp, f) for dp, dn, filenames in os.walk(rootdir) for f in filenames if f.endswith('.csv')]   

    data_total = {}    
    for file_path in filelist:    
        file_name = os.path.basename(file_path)    
        print("file_name :", file_name)   
        
        str_list = file_name.split("-")   
        trial_date = str_list[0]
        trial_time = str_list[1]
        subject_name = str_list[2]
        activity_name = str_list[3]
        speed_name = str_list[4]
        # print("str_list :", str_list[5][])   
        # trial_idx = int(str_list[5][6:])   

        # Read in logged data
        data = pd.read_csv(file_path, header=1).to_numpy()  

        # Process data
        torque_left = -data[:, 4] * torque_gain * all_sign_inverse
        torque_right = -data[:, 5] * torque_gain * all_sign_inverse
        hip_angle_left = -data[:, 0] * all_sign_inverse
        hip_angle_right = -data[:, 1] * all_sign_inverse
        hip_angular_velocity_left = -data[:, 2] * all_sign_inverse
        hip_angular_velocity_right = -data[:, 3] * all_sign_inverse

        # Make the same length
        data_length = min(len(torque_left), len(torque_right), 
                        len(hip_angle_left), len(hip_angle_right),
                        len(hip_angular_velocity_left), len(hip_angular_velocity_right)
                        )  

        torque_left = torque_left[:data_length]  
        torque_right = torque_right[:data_length] * right_leg_sign  
        hip_angle_left = hip_angle_left[:data_length] 
        hip_angle_right = hip_angle_right[:data_length]  
        hip_angular_velocity_left = hip_angular_velocity_left[:data_length]
        hip_angular_velocity_right = hip_angular_velocity_right[:data_length]  
        
        hip_power_left = torque_left * hip_angular_velocity_left / 180 * np.pi
        hip_power_right = torque_right * hip_angular_velocity_right / 180 * np.pi

        # Get a filtered hip angle to help gait segmentation
        fs = 1 / dt_controller  # Hz
        fc = 10  # cutoff frequency
        b, a = butter(2, fc / (fs / 2), btype='low')

        hip_angle_filtered_left = filtfilt(b, a, hip_angle_left)
        hip_angle_filtered_right = filtfilt(b, a, hip_angle_right)

        # Segment gait
        min_peak_height = 24
        if speed_name == 'S2x00':
            min_gait_duration = int(np.floor(0.22 / dt_controller))
            max_gait_duration = int(np.ceil(2 / dt_controller))
        else:
            min_gait_duration = int(np.floor(0.4 / dt_controller))
            max_gait_duration = int(np.ceil(2 / dt_controller))

        # Subject-specific settings
        if trial_date == '20240830' and speed_name == 'S2x00':
            min_gait_duration = int(np.floor(0.25 / dt_controller))
        if trial_date == '20240830':
            min_peak_height = 20
        if trial_date == '20240905' and speed_name == 'S2x00':
            min_peak_height = 45
        if trial_date == '20240905' and subject_name == 'Yuming' and speed_name == 'S2x00':
            min_peak_height = 30
            min_gait_duration = int(np.floor(0.30 / dt_controller))

        # Left leg
        gait_head_left_list = []
        gait_tail_left_list = []
        peaks, _ = find_peaks(hip_angle_filtered_left, height=min_peak_height, distance=min_gait_duration)

        for gait_idx in range(5, len(peaks) - 3):
            gait_head = peaks[gait_idx]
            gait_tail = peaks[gait_idx + 1]
            if gait_tail - gait_head <= max_gait_duration:
                gait_head_left_list.append(gait_head)
                gait_tail_left_list.append(gait_tail)

        gait_duration_left_list = np.array(gait_tail_left_list) - np.array(gait_head_left_list)
        gait_duration_left_mean = np.mean(gait_duration_left_list)
        gait_duration_left_std = np.std(gait_duration_left_list)
        print(f'Left leg: {len(gait_head_left_list)} candidate gaits found. Mean = {gait_duration_left_mean}, std = {gait_duration_left_std}')

        gait_head_right_list = []
        gait_tail_right_list = []
        peaks, _ = find_peaks(hip_angle_filtered_right, height=min_peak_height, distance=min_gait_duration)

        for gait_idx in range(5, len(peaks) - 3):
            gait_head = peaks[gait_idx]
            gait_tail = peaks[gait_idx + 1]
            if gait_tail - gait_head <= max_gait_duration:
                gait_head_right_list.append(gait_head)
                gait_tail_right_list.append(gait_tail)

        gait_duration_right_list = np.array(gait_tail_right_list) - np.array(gait_head_right_list)
        gait_duration_right_mean = np.mean(gait_duration_right_list)
        gait_duration_right_std = np.std(gait_duration_right_list)
        print(f'Right leg: {len(gait_head_right_list)} candidate gaits found. Mean = {gait_duration_right_mean}, std = {gait_duration_right_std}')

        if subject_name not in data_total:
            data_total[subject_name] = {}
        if activity_name not in data_total[subject_name]:
            data_total[subject_name][activity_name] = {}
        if speed_name not in data_total[subject_name][activity_name]:
            data_total[subject_name][activity_name][speed_name] = {}

        data_total[subject_name][activity_name][speed_name] = {
            'torqueLeftList': torque_left,
            'torqueRightList': torque_right,
            'hipAngleLeftList': hip_angle_left,
            'hipAngleRightList': hip_angle_right,
            'hipAngleFilteredLeftList': hip_angle_filtered_left,
            'hipAngleFilteredRightList': hip_angle_filtered_right,
            'hipAngularVelocityLeftList': hip_angular_velocity_left,
            'hipAngularVelocityRightList': hip_angular_velocity_right,
            'hipPowerLeftList': hip_power_left,
            'hipPowerRightList': hip_power_right,
            'gaitHeadLeftList': gait_head_left_list,
            'gaitTailLeftList': gait_tail_left_list,
            'gaitHeadRightList': gait_head_right_list,
            'gaitTailRightList': gait_tail_right_list
        }

    return data_total  


def plot_feedback_data(
    data_total   = None, 
    subject_name = None, 
    dt_logging   = None 
):  
    ######################## Create a figure ############################### 
    fig, axs = plt.subplots(4, 2, figsize=(12, 12), constrained_layout=True)
    fig.patch.set_facecolor('w')  
    
    for idx in range(len(activity_name_list)):
        activity_name = activity_name_list[idx]
        speed_name = speed_name_list[idx]
        speed_display_name = speed_display_name_list[idx]
        
        # Retrieve data
        hip_angle_left_list = data_total[subject_name][activity_name][speed_name]['hipAngleLeftList']
        hip_angle_right_list = data_total[subject_name][activity_name][speed_name]['hipAngleRightList']
        hip_angle_filtered_left_list = data_total[subject_name][activity_name][speed_name]['hipAngleFilteredLeftList']
        hip_angle_filtered_right_list = data_total[subject_name][activity_name][speed_name]['hipAngleFilteredRightList']
        torque_left_list = data_total[subject_name][activity_name][speed_name]['torqueLeftList']
        torque_right_list = data_total[subject_name][activity_name][speed_name]['torqueRightList']
        gait_head_left_list = data_total[subject_name][activity_name][speed_name]['gaitHeadLeftList']
        gait_tail_left_list = data_total[subject_name][activity_name][speed_name]['gaitTailLeftList']
        gait_head_right_list = data_total[subject_name][activity_name][speed_name]['gaitHeadRightList']
        gait_tail_right_list = data_total[subject_name][activity_name][speed_name]['gaitTailRightList']
        
        time_list = np.arange(len(hip_angle_left_list)) * dt_logging

        # Plot for Left leg
        ax_left = axs[idx, 0]
        ax_left.plot(time_list, hip_angle_left_list, label='Raw')
        ax_left.plot(time_list, hip_angle_filtered_left_list, label='Filtered')
        ax_left.plot(time_list[gait_head_left_list], hip_angle_left_list[gait_head_left_list], 'go', label='Gait Heads')
        ax_left.set_ylabel('Hip Angle (deg)')
        ax_left.legend(loc='best', frameon=False)
        
        ax_torque_left = ax_left.twinx()
        ax_torque_left.plot(time_list, torque_left_list, 'r-', label='Command Torque (Nm)')
        ax_torque_left.set_ylabel('Command Torque (Nm)')
        
        if idx == len(activity_name_list) - 1:
            ax_left.set_xlabel('Time (sec)')
        if idx == 0:
            ax_left.set_title(f'Left leg - {activity_name} @ {speed_display_name}')
        else:
            ax_left.set_title(f'{activity_name} @ {speed_display_name}')

        # Plot for Right leg
        ax_right = axs[idx, 1]
        ax_right.plot(time_list, hip_angle_right_list, label='Raw')
        ax_right.plot(time_list, hip_angle_filtered_right_list, label='Filtered')
        ax_right.plot(time_list[gait_head_right_list], hip_angle_right_list[gait_head_right_list], 'go', label='Gait Heads')
        ax_right.set_ylabel('Hip Angle (deg)')
        ax_right.legend(loc='best', frameon=False)
        
        ax_torque_right = ax_right.twinx()
        ax_torque_right.plot(time_list, torque_right_list, 'r-', label='Command Torque (Nm)')
        ax_torque_right.set_ylabel('Command Torque (Nm)')
        
        if idx == len(activity_name_list) - 1: 
            ax_right.set_xlabel('Time (sec)')
        if idx == 0:
            ax_right.set_title(f'Right leg - {activity_name} @ {speed_display_name}')
        else:
            ax_right.set_title(f'{activity_name} @ {speed_display_name}')

    plt.show()
    #########################################################################  


def plot_hip_angle_power(
        data_total   = None,
        subject_name = None, 
        dt_logging   = None 
    ):    
    # Create a new figure for power and hip angle
    fig, axs = plt.subplots(4, 2, figsize=(12, 12), constrained_layout=True)
    fig.patch.set_facecolor('w')    

    for idx in range(len(activity_name_list)):  
        activity_name = activity_name_list[idx] 
        speed_name = speed_name_list[idx]  
        speed_display_name = speed_display_name_list[idx]  
        
        # Retrieve data  
        hip_angle_left_list = data_total[subject_name][activity_name][speed_name]['hipAngleLeftList']
        hip_angle_right_list = data_total[subject_name][activity_name][speed_name]['hipAngleRightList']
        hip_power_left_list = data_total[subject_name][activity_name][speed_name]['hipPowerLeftList']
        hip_power_right_list = data_total[subject_name][activity_name][speed_name]['hipPowerRightList']
        
        time_list = np.arange(len(hip_angle_left_list)) * dt_logging

        # Plot for Left leg
        ax_left = axs[idx, 0]
        ax_left.plot(time_list, hip_angle_left_list, label='Hip Angle (deg)')
        ax_left.set_ylabel('Hip Angle (deg)')
        
        ax_power_left = ax_left.twinx()
        ax_power_left.plot(time_list, hip_power_left_list, 'r-', label='Power (W)')
        ax_power_left.set_ylabel('Power (W)')
        
        if idx == len(activity_name_list) - 1:
            ax_left.set_xlabel('Time (sec)')
        if idx == 0:
            ax_left.set_title(f'Left leg - {activity_name} @ {speed_display_name}')
        else:
            ax_left.set_title(f'{activity_name} @ {speed_display_name}')

        # Plot for Right leg
        ax_right = axs[idx, 1]
        ax_right.plot(time_list, hip_angle_right_list, label='Hip Angle (deg)')
        ax_right.set_ylabel('Hip Angle (deg)')
        
        ax_power_right = ax_right.twinx()
        ax_power_right.plot(time_list, hip_power_right_list, 'r-', label='Power (W)')
        ax_power_right.set_ylabel('Power (W)')
        
        if idx == len(activity_name_list) - 1:
            ax_right.set_xlabel('Time (sec)')
        if idx == 0:
            ax_right.set_title(f'Right leg - {activity_name} @ {speed_display_name}')
        else:
            ax_right.set_title(f'{activity_name} @ {speed_display_name}')

    plt.show()   


def plot_average_torque(
    data_total    = None, 
    subject_name  = None,
    dt_controller = None 
):    
    normalized_gait_length = 101
    normalized_gait_cycle_list = np.arange(normalized_gait_length)

    for idx in range(len(activity_name_list)):
        activity_name = activity_name_list[idx]
        speed_name = speed_name_list[idx]
        speed_display_name = speed_display_name_list[idx]
        
        # Retrieve data for the current activity and speed
        hip_angle_left_list = data_total[subject_name][activity_name][speed_name]['hipAngleLeftList']
        hip_angle_right_list = data_total[subject_name][activity_name][speed_name]['hipAngleRightList']
        torque_left_list = data_total[subject_name][activity_name][speed_name]['torqueLeftList']
        torque_right_list = data_total[subject_name][activity_name][speed_name]['torqueRightList']
        hip_power_left_list = data_total[subject_name][activity_name][speed_name]['hipPowerLeftList']
        hip_power_right_list = data_total[subject_name][activity_name][speed_name]['hipPowerRightList']
        gait_head_left_list = data_total[subject_name][activity_name][speed_name]['gaitHeadLeftList']
        gait_tail_left_list = data_total[subject_name][activity_name][speed_name]['gaitTailLeftList']
        gait_head_right_list = data_total[subject_name][activity_name][speed_name]['gaitHeadRightList']
        gait_tail_right_list = data_total[subject_name][activity_name][speed_name]['gaitTailRightList']
        
        # Compute average quantities for the left leg
        input_data = {
            'gaitHeadList': gait_head_left_list,
            'gaitTailList': gait_tail_left_list,
            'dataSeq': [torque_left_list, hip_power_left_list],
            'dataOutNames': ['Torque', 'HipPower'],
            'dataShift': 0,
            'normalizedGaitLength': normalized_gait_length
        }
        result_left = compute_gait_average_profile(input_data)
        
        # Compute mechanical work for the left leg
        injected_work_left_list = []
        for gait_idx in range(len(gait_head_left_list)):
            gait_head = gait_head_left_list[gait_idx]
            gait_tail = gait_tail_left_list[gait_idx]
            power_list = hip_power_left_list[gait_head:gait_tail]
            injected_work = np.trapz(power_list, dx=dt_controller)
            injected_work_left_list.append(injected_work)
        
        # Store results for left leg
        data_total[subject_name][activity_name][speed_name]['averagedMetricsLeft'] = result_left
        data_total[subject_name][activity_name][speed_name]['injectedWorkLeftList'] = injected_work_left_list

        # Compute average quantities for the right leg
        input_data['gaitHeadList'] = gait_head_right_list
        input_data['gaitTailList'] = gait_tail_right_list
        input_data['dataSeq'] = [torque_right_list, hip_power_right_list]
        result_right = compute_gait_average_profile(input_data)
        
        # Compute mechanical work for the right leg
        injected_work_right_list = []
        for gait_idx in range(len(gait_head_right_list)):
            gait_head = gait_head_right_list[gait_idx]
            gait_tail = gait_tail_right_list[gait_idx]
            power_list = hip_power_right_list[gait_head:gait_tail]
            injected_work = np.trapz(power_list, dx=dt_controller)
            injected_work_right_list.append(injected_work)

        # Store results for right leg
        data_total[subject_name][activity_name][speed_name]['averagedMetricsRight'] = result_right
        data_total[subject_name][activity_name][speed_name]['injectedWorkRightList'] = injected_work_right_list
 
    ###################################################################
    fig, axs = plt.subplots(4, 2, figsize=(15, 10))
    fig.patch.set_facecolor('white')

    normalized_gait_cycle_list = np.arange(101)  

    for idx in range(len(activityNameList)):
        activity_name = activityNameList[idx]
        speed_name = speedNameList[idx]
        speed_display_name = speedDisplayNameList[idx]

        averaged_metrics_left = dataTotal[subjectName][activity_name][speed_name]['averagedMetricsLeft']
        averaged_metrics_right = dataTotal[subjectName][activity_name][speed_name]['averagedMetricsRight']
        injected_work_left_list = dataTotal[subjectName][activity_name][speed_name]['injectedWorkLeftList']
        injected_work_right_list = dataTotal[subjectName][activity_name][speed_name]['injectedWorkRightList']

        # Left leg metrics
        gait_torque_left_avg = averaged_metrics_left['gaitTorqueNormalizedAvgList']
        gait_hip_power_left_avg = averaged_metrics_left['gaitHipPowerNormalizedAvgList']
        gait_torque_left_std = averaged_metrics_left['gaitTorqueNormalizedStdList']
        gait_hip_power_left_std = averaged_metrics_left['gaitHipPowerNormalizedStdList']

        # Right leg metrics
        gait_torque_right_avg = averaged_metrics_right['gaitTorqueNormalizedAvgList']
        gait_hip_power_right_avg = averaged_metrics_right['gaitHipPowerNormalizedAvgList']
        gait_torque_right_std = averaged_metrics_right['gaitTorqueNormalizedStdList']
        gait_hip_power_right_std = averaged_metrics_right['gaitHipPowerNormalizedStdList']

        # Left leg plot
        ax = axs[idx, 0]
        peak_torque_left = np.max(np.abs(gait_torque_left_avg))
        rms_torque_left = np.sqrt(np.mean(gait_torque_left_avg**2))
        peak_power_left = np.max(np.abs(gait_hip_power_left_avg))
        rms_power_left = np.sqrt(np.mean(gait_hip_power_left_avg**2))

        ax.fill_between(normalized_gait_cycle_list, 
                        gait_torque_left_avg - gait_torque_left_std, 
                        gait_torque_left_avg + gait_torque_left_std, 
                        color='blue', alpha=0.1)
        ax.plot(normalized_gait_cycle_list, gait_torque_left_avg, label='Avg Torque')
        ax.set_ylabel('Command Torque (Nm)')
        ax.tick_params(axis='y')

        ax2 = ax.twinx()
        ax2.fill_between(normalized_gait_cycle_list, 
                        gait_hip_power_left_avg - gait_hip_power_left_std, 
                        gait_hip_power_left_avg + gait_hip_power_left_std, 
                        color='orange', alpha=0.1)
        ax2.plot(normalized_gait_cycle_list, gait_hip_power_left_avg, color='orange', label='Avg Power')
        ax2.set_ylabel('Power (W)')

        ax.set_title(f'Left leg: {activity_name} @ {speed_display_name}, '
                    f'Peak Torque = {peak_torque_left:.2f} Nm, RMS Torque = {rms_torque_left:.2f} Nm,\n'
                    f'Peak Power = {peak_power_left:.2f} W, RMS Power = {rms_power_left:.2f} W, '
                    f'Mean Work = {np.mean(injected_work_left_list):.2f} J', fontsize=font_size)

        # Right leg plot
        ax = axs[idx, 1]  
        peak_torque_right = np.max(np.abs(gait_torque_right_avg))
        rms_torque_right = np.sqrt(np.mean(gait_torque_right_avg**2))
        peak_power_right = np.max(np.abs(gait_hip_power_right_avg))
        rms_power_right = np.sqrt(np.mean(gait_hip_power_right_avg**2)) 

        ax.fill_between(
            normalized_gait_cycle_list, 
            gait_torque_right_avg - gait_torque_right_std, 
            gait_torque_right_avg + gait_torque_right_std, 
            color='blue', 
            alpha=0.1
        )  
        
        ax.plot(normalized_gait_cycle_list, gait_torque_right_avg, label='Avg Torque')
        ax.set_ylabel('Command Torque (Nm)')  
        ax.tick_params(axis='y')   

        ax2 = ax.twinx()
        ax2.fill_between(
            normalized_gait_cycle_list, 
            gait_hip_power_right_avg - gait_hip_power_right_std, 
            gait_hip_power_right_avg + gait_hip_power_right_std, 
            color='orange', 
            alpha=0.1
        )
        ax2.plot(normalized_gait_cycle_list, gait_hip_power_right_avg, color='orange', label='Avg Power')
        ax2.set_ylabel('Power (W)')  

        ax.set_title(f'Right leg: {activity_name} @ {speed_display_name}, '
                    f'Peak Torque = {peak_torque_right:.2f} Nm, RMS Torque = {rms_torque_right:.2f} Nm,\n'
                    f'Peak Power = {peak_power_right:.2f} W, RMS Power = {rms_power_right:.2f} W, '
                    f'Mean Work = {np.mean(injected_work_right_list):.2f} J', fontsize=font_size)  

    plt.tight_layout()
    plt.show()  



if __name__ == "__main__":    
    # Create the parser 
    parser = argparse.ArgumentParser(description='A simple calculator.')   
    
    parser.add_argument('index', type=int, help='The first number')  

    args = parser.parse_args()

    ################################# 
    # Define activity and speed names
    activity_name_list = ['Walk', 'Walk', 'Walk', 'Run']   
    speed_name_list = ['S0x75', 'S1x25', 'S1x75', 'S2x00']   
    speed_display_name_list = ['0.75 m/s', '1.25 m/s', '1.75 m/s', '2.00 m/s']  
    #################################   

    activityNameList     = activity_name_list 
    speedNameList        = speed_name_list   
    speedDisplayNameList = speed_display_name_list  
    subjectName          = "Yuming"   

    dataTotal            = process_data(
        rootdir = "./data/Controller_100Hz_Gain_0x20_20240905_Yuming", 
        dt_controller = 1/100, 
        dt_logging = 1/100, 
        torque_gain = 0.20/20, 
        right_leg_sign = -1, 
        all_sign_inverse = 1  
    )
    
    
    if args.index == 0:  
        plot_feedback_data(
            data_total   = dataTotal, 
            subject_name = subjectName, 
            dt_logging   = 1/100 
        )
    
    if args.index == 1: 
        plot_hip_angle_power(
            data_total   = dataTotal,  
            subject_name = subjectName,  
            dt_logging   = 1/100 
        )
        
    if args.index == 2: 
        plot_average_torque(
            data_total   = dataTotal, 
            subject_name = subjectName, 
            dt_controller= 1/100
        )  
