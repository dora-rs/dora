# Dora pipeline Robots

AlexK Low Cost Robot is a low-cost robotic arm that can be teleoperated using a similar arm. This repository contains
the Dora pipeline to manipulate the arms, the camera, and record/replay episodes with LeRobot.

## Recording

This section explains how to record episodes for LeRobot using the AlexK Low Cost Robot.

Recording is the process of tele operating the robot and saving the episodes to a dataset. The dataset is used to train
the robot to perform tasks autonomously.

To record episodes with Dora, you have to configure the Dataflow `record_mono_teleop_real.yml` file to integrate the
arms and the camera. The graph file is located in the `graphs` folder.

Make sure to:

- Adjust the serial ports of `lcr-leader` and `lcr-follower` in the `record_mono_teleop_real.yml` file.
- Adjust the camera PATH in the `record_mono_teleop_real.yml` file.
- Adjust image and video WIDTH and HEIGHT in the `record_mono_teleop_real.yml` file, if needed.
- Adjust recording framerate with your camera framerate in the `record_mono_teleop_real.yml` file.
- Adjust CONFIG path environment variables in the `record_mono_teleop_real.yml` file for both arms if needed.
- Adjust `LEADER_CONTROL` and `FOLLOWER_CONTROL` environment variables in the `record_mono_teleop_real.yml` file if
  needed.

You can now start the Dora pipeline to record episodes for LeRobot:

```bash
cd dora-lerobot

# If you are using a custom environment, you will have to activate it before running the command
source [your_custom_env_bin]/activate

# If you followed the installation instructions, you can run the following command
source venv/bin/activate # On Linux
source venv/Scripts/activate # On Windows bash
venv\Scripts\activate.bat # On Windows cmd
venv\Scripts\activate.ps1 # On Windows PowerShell

dora build ./robots/alexk-lcr/graphs/record_mono_teleop_real.yml # Only the first time, it will install all the requirements if needed

dora up
dora start ./robots/alexk-lcr/graphs/record_mono_teleop_real.yml
```

Then, you can tele operate the follower with the leader. A window will pop up showing the camera feed, and some text.

1. Press space to start/stop recording
2. Press return if you want to tell the recording that you failed the current episode, or the previous episode if you
   have not started the current one
3. Close the window to stop the recording
4. Write down the location of the logs (e.g `018fc3a8-3b76-70f5-84a2-22b84df24739`), this is where the
   dataset (and logs) are stored.

You can now use our script to convert the logs to an understandable dataset:

```bash
cd dora-lerobot

# If you are using a custom environment, you will have to activate it before running the command
source [your_custom_env_bin]/activate

# If you followed the installation instructions, you can run the following command
source venv/bin/activate # On Linux
source venv/Scripts/activate # On Windows bash
venv\Scripts\activate.bat # On Windows cmd
venv\Scripts\activate.ps1 # On Windows PowerShell

python ./datasets/build_dataset.py --record-path [path_to_recorded_logs] --dataset-name [dataset_name] --framerate [framerate]
```

**Note:** On default, the framerate is 30. If you have recorded with a different framerate, you will have to adjust it.

## The dora graph

[![](https://mermaid.ink/img/pako:eNqdVMFu2zAM_RVB57berjn0MOzaU3eLCoGR6ESobBqSnK4o-u-j5NizE6Np64NBPfE9Us-03qQhi3IjxempPb2YA4Qk_vxSrRDeBO0RLIZx5dqEoSMPCUeoJs-0IYU6bM1RG2gwQAaOziJpBmkUwUA7SjqgoWAzYinAabmtZgulnlQb-90-QHcQWuuyp7XY5uApU-e7yXHN0zsnlahkDSWqAlSN897F6uePrVJTXJU8bLmf8jpvU9zeiuTMs4Aout573VF0yVHLG_crLo2WZN6UytwRv-Sv-DpInksM6HWBi_75YFPy_JN99aQL7rJwJu8JZiRWeQkuoV7C3-jDNbDHQryYsZWzdi7ywGXyKeQ2Lf4t_IuRXAhm-v9an83lQiXgj1an4Xirc34-hNMx1ymf8RfMZOn85_m6L1fZNTiPtsxxifRVjYV9C7doFzEcIbd-V8B4x57qvltt5YNfai4U02DSQkDeSLa8AWf5onvLckqmAzao5IZDC-FZSdW-cx70iR5fWyM3KfR4IwP1-4Pc1OAjr_rOsvxvB3zlNBOK1iUKD8M9Wq7T93-SiOfx?type=png)](https://mermaid.live/edit#pako:eNqdVMFu2zAM_RVB57berjn0MOzaU3eLCoGR6ESobBqSnK4o-u-j5NizE6Np64NBPfE9Us-03qQhi3IjxempPb2YA4Qk_vxSrRDeBO0RLIZx5dqEoSMPCUeoJs-0IYU6bM1RG2gwQAaOziJpBmkUwUA7SjqgoWAzYinAabmtZgulnlQb-90-QHcQWuuyp7XY5uApU-e7yXHN0zsnlahkDSWqAlSN897F6uePrVJTXJU8bLmf8jpvU9zeiuTMs4Aout573VF0yVHLG_crLo2WZN6UytwRv-Sv-DpInksM6HWBi_75YFPy_JN99aQL7rJwJu8JZiRWeQkuoV7C3-jDNbDHQryYsZWzdi7ywGXyKeQ2Lf4t_IuRXAhm-v9an83lQiXgj1an4Xirc34-hNMx1ymf8RfMZOn85_m6L1fZNTiPtsxxifRVjYV9C7doFzEcIbd-V8B4x57qvltt5YNfai4U02DSQkDeSLa8AWf5onvLckqmAzao5IZDC-FZSdW-cx70iR5fWyM3KfR4IwP1-4Pc1OAjr_rOsvxvB3zlNBOK1iUKD8M9Wq7T93-SiOfx)

## License

This library is licensed under the [Apache License 2.0](../../LICENSE).