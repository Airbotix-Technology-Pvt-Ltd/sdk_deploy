import mujoco
import mujoco.viewer
import time
import numpy as np
import os

# Set working directory to the script's directory to find the XML
os.chdir(os.path.dirname(os.path.abspath(__file__)))

def main():
    # Load the model
    model = mujoco.MjModel.from_xml_path('lite3_lidar.xml')
    data = mujoco.MjData(model)

    # Lidar sensor names
    lidar_names = [f'lidar_{i}' for i in range(8)]
    lidar_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name) for name in lidar_names]

    print("Launching MuJoCo viewer. Press ESC to exit.")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Print lidar data every 100 steps
            if data.time % 0.5 < 0.005: 
                distances = [data.sensordata[model.sensor_adr[id]] for id in lidar_ids]
                print(f"Time: {data.time:.2f}s | Lidar Distances: {[f'{d:.2f}' for d in distances]}")

            # Sync viewer
            viewer.sync()
            
            # Maintain real-time simulation speed
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
