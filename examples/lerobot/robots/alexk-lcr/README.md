# Dora pipeline Robots

AlexK Low Cost Robot is a low-cost robotic arm that can be teleoperated using a similar arm. This repository contains
the Dora pipeline to record episodes for LeRobot.

## Assembling

Check the [ASSEMBLING.md](ASSEMBLING.md) file for instructions on how to assemble the robot from scratch using the
provided parts from the [AlexK Low Cost Robot](https://github.com/AlexanderKoch-Koch/low_cost_robot)

## Installation

Check the [INSTALLATION.md](INSTALLATION.md) file for instructions on how to install the required software and
environment
to run the robot.

## Configuring

Check the [CONFIGURING.md](CONFIGURING.md) file for instructions on how to configure the robot to record episodes for
LeRobot and teleoperate the robot.

## Recording

It's probably better to check the [examples](#examples) below before trying to record episodes. It will give you a
better
understanding of how Dora works.

Check the [RECORDING.md](RECORDING.md) file for instructions on how to record episodes for LeRobot.

## Examples

There are also some other example applications in the `graphs` folder. Have fun!

Here is a list of the available examples:

- `mono_teleop_real.yml`: A simple real teleoperation pipeline that allows you to control a follower arm using a leader
  arm. It
  does not record the episodes, so you don't need to have a camera.

You must configure the arms, retrieve the device port, and modify the file `mono_teleop_real.yml` to set the correct
environment variables. (e.g. `PORT` and `CONFIG`, `LEADER_CONTROL` and `FOLLOWER_CONTROL`)

```bash
cd dora-lerobot/

# If you are using a custom environment, you will have to activate it before running the command
source [your_custom_env_bin]/activate

# If you followed the installation instructions, you can run the following command
source venv/bin/activate # On Linux
source venv/Scripts/activate # On Windows bash
venv\Scripts\activate.bat # On Windows cmd
venv\Scripts\activate.ps1 # On Windows PowerShell

dora build ./robots/alexk-lcr/graphs/mono_teleop_real.yml # Only the first time, it will install all the requirements if needed

dora up
dora start ./robots/alexk-lcr/graphs/mono_teleop_real.yml
```

[![](https://mermaid.ink/img/pako:eNqVUsFOxCAQ_RUy591Urz14MF496W0xZCzTlkihmUI2ZrP_LtDtutomRg4w83jvMcCcoPGaoAZxGa31x6ZHDuL1UTohbMPKEmriJTMuEI_eYqAFar1NskyZ4nvHOPZCKaU9Y1rEIQdvmXu7G8xAfJkzqUSFJUQWVAWoBmOtmar7u4OU17gqPHJaujJtK8R-L8ZorRr9ZILxLgEPGxdaqi_8hYqTWPC1fuMJZsvfFjP6p8H_qv9-7dWHZFHn8UaUijiyCaR-wmsv2EE6f0CjUzecsreE0NNAEuoUauQPCdKdEw9j8C-froE6cKQdsI9dD3WLdkpZHHWq5Mlg-urhipI2wfPz3Gyl585fka3hkA?type=png)](https://mermaid.live/edit#pako:eNqVUsFOxCAQ_RUy591Urz14MF496W0xZCzTlkihmUI2ZrP_LtDtutomRg4w83jvMcCcoPGaoAZxGa31x6ZHDuL1UTohbMPKEmriJTMuEI_eYqAFar1NskyZ4nvHOPZCKaU9Y1rEIQdvmXu7G8xAfJkzqUSFJUQWVAWoBmOtmar7u4OU17gqPHJaujJtK8R-L8ZorRr9ZILxLgEPGxdaqi_8hYqTWPC1fuMJZsvfFjP6p8H_qv9-7dWHZFHn8UaUijiyCaR-wmsv2EE6f0CjUzecsreE0NNAEuoUauQPCdKdEw9j8C-froE6cKQdsI9dD3WLdkpZHHWq5Mlg-urhipI2wfPz3Gyl585fka3hkA)

- `bi_teleop_real.yml`: A simple real tele operation pipeline that allows you to control two follower arm using two
  leader arm
  (left and right). It does not record the episodes, so you don't need to have a camera.

You must configure the arms, retrieve the device port, and modify the file `bi_teleop_real.yml` to set the correct
environment variables. (e.g. `PORT` and `CONFIG`)

```bash
cd dora-lerobot/

# If you are using a custom environment, you will have to activate it before running the command
source [your_custom_env_bin]/activate

# If you followed the installation instructions, you can run the following command
source venv/bin/activate # On Linux
source venv/Scripts/activate # On Windows bash
venv\Scripts\activate.bat # On Windows cmd
venv\Scripts\activate.ps1 # On Windows PowerShell

dora build ./robots/alexk-lcr/graphs/bi_teleop_real.yml # Only the first time, it will install all the requirements if needed

dora up
dora start ./robots/alexk-lcr/graphs/bi_teleop_real.yml
```

[![](https://mermaid.ink/img/pako:eNqlVMFugzAM_ZUo51ZsVw47TLvutN2aKsqIgWghQSZRNVX99yWhtAXBNjoOxrz4vdgmzpEWVgLNKTk_pbaHohboyPszM4ToArmG0gUjJOAIUsYBtlYLByO8tDqoXINRVfVUoMdmFPqFq0TnPyoUbU0459KiCC-yi84-Mm5XnWoAzzYGJS9FERIJWQKyRmmtuuzxYcfYxc9SHBjJTDLzDLLdktZrzVvbKaesCcDTjy0a6kjMgSQ6MuALSkud7XeYivXo36TuKGv6O6eykV5ZcUMPOR1QOeBjeFF1XVLLx2l9t385huv6PSt2T23zA_Sflk916YaGjBqhZJj9Y9yHUVdDA4zmwZUCPxll5hTihHf27csUNHfoYUPR-qqmeSl0F758K0M-L0qEMWwuKEjlLL72V0u6YU7fOOqbHg?type=png)](https://mermaid.live/edit#pako:eNqlVMFugzAM_ZUo51ZsVw47TLvutN2aKsqIgWghQSZRNVX99yWhtAXBNjoOxrz4vdgmzpEWVgLNKTk_pbaHohboyPszM4ToArmG0gUjJOAIUsYBtlYLByO8tDqoXINRVfVUoMdmFPqFq0TnPyoUbU0459KiCC-yi84-Mm5XnWoAzzYGJS9FERIJWQKyRmmtuuzxYcfYxc9SHBjJTDLzDLLdktZrzVvbKaesCcDTjy0a6kjMgSQ6MuALSkud7XeYivXo36TuKGv6O6eykV5ZcUMPOR1QOeBjeFF1XVLLx2l9t385huv6PSt2T23zA_Sflk916YaGjBqhZJj9Y9yHUVdDA4zmwZUCPxll5hTihHf27csUNHfoYUPR-qqmeSl0F758K0M-L0qEMWwuKEjlLL72V0u6YU7fOOqbHg)

- `mono_teleop_simu.yml`: A simple simulation tele operation pipeline that allows you to control a simulated follower
  arm using a leader arm. It does not record the episodes, so you don't need to have a camera.

You must configure the arms, retrieve the device port, and modify the file `mono_teleop_simu.yml` to set the correct
environment variables. (e.g. `PORT` and `CONFIG`)

```bash
cd dora-lerobot/


# If you are using a custom environment, you will have to activate it before running the command
source [your_custom_env_bin]/activate

# If you followed the installation instructions, you can run the following command
source venv/bin/activate # On Linux
source venv/Scripts/activate # On Windows bash
venv\Scripts\activate.bat # On Windows cmd
venv\Scripts\activate.ps1 # On Windows PowerShell

dora build ./robots/alexk-lcr/graphs/mono_teleop_simu.yml # Only the first time, it will install all the requirements if needed

dora up
dora start ./robots/alexk-lcr/graphs/mono_teleop_simu.yml
```

[![](https://mermaid.ink/img/pako:eNp1UstuwyAQ_JUV50Rurz70UPXaU3sLFdqatY2CwcKgqIry711w4ubhcoDdYWZ3eBxF4zWJWsB5tNYfmh5DhM9X6QBsE5Ql1BQumXGRwugtRrpArbcsy5QpfXcBxx6UUtoH5AV2OfjK3OvdaAYK5zmTSlRYAFlQFaAajLVmqp6fdlIucVV45LR0Zbp1AdstRNPsAScYk7Vq9JOJxjveeFk50Jxl1UJk5Yw-au-Ov2a1lFpt_HdR_yuL9TXBXffM7TxedWHXh2AiqVv4sZbYCG47oNH88sdcW4rY00BS1BxqDHsppDsxD1P0Hz-uEXUMiTYi-NT1om7RTpylUbOTN4P8rMOCkjbRh_f5Y5X_dfoF5ZjY9g?type=png)](https://mermaid.live/edit#pako:eNp1UstuwyAQ_JUV50Rurz70UPXaU3sLFdqatY2CwcKgqIry711w4ubhcoDdYWZ3eBxF4zWJWsB5tNYfmh5DhM9X6QBsE5Ql1BQumXGRwugtRrpArbcsy5QpfXcBxx6UUtoH5AV2OfjK3OvdaAYK5zmTSlRYAFlQFaAajLVmqp6fdlIucVV45LR0Zbp1AdstRNPsAScYk7Vq9JOJxjveeFk50Jxl1UJk5Yw-au-Ov2a1lFpt_HdR_yuL9TXBXffM7TxedWHXh2AiqVv4sZbYCG47oNH88sdcW4rY00BS1BxqDHsppDsxD1P0Hz-uEXUMiTYi-NT1om7RTpylUbOTN4P8rMOCkjbRh_f5Y5X_dfoF5ZjY9g)

- `mono_teleop_real_and_simu.yml`: A simple real and simulation tele operation pipeline that allows you to control a
  simulated and real follower arm using a real leader arm. It does not record the episodes, so you don't need to have a
  camera.

You must configure the arms, retrieve the device port, and modify the file `mono_teleop_real_and_simu.yml` to set the
correct
environment variables. (e.g. `PORT` and `CONFIG`)

```bash
cd dora-lerobot/


# If you are using a custom environment, you will have to activate it before running the command
source [your_custom_env_bin]/activate

# If you followed the installation instructions, you can run the following command
source venv/bin/activate # On Linux
source venv/Scripts/activate # On Windows bash
venv\Scripts\activate.bat # On Windows cmd
venv\Scripts\activate.ps1 # On Windows PowerShell

dora build ./robots/alexk-lcr/graphs/mono_teleop_real_and_simu.yml # Only the first time, it will install all the requirements if needed

dora up
dora start ./robots/alexk-lcr/graphs/mono_teleop_real_and_simu.yml
```

[![](https://mermaid.ink/img/pako:eNqdU8luwyAQ_RXEOZHbqw89VL321N5ChajBMQqLxaKoivLvHXCM3IS0lX3Aw-O9YRbmhDvLBW4xuny9ssduYC6g92diEFKdo0owLty8kyYIN1rFgpih3iqQVSnUSx2veRfQx8-9Y-OAKKXcOgY_tEvGRxIsT4PUoJrWRMpWZiGUBE0GGi2Vkr55fNgRUuwm84ThxOSlEgrablGQ3QExj8aoFB2tl0FaAwdPlRLM4qQrVNAWpzf6StEml9cuJvRfDm5SgPQKf9mSWoXyvdVUf2lmEu0tW4gg4qOT0Oaf8D1fq3Muz2hdLn_Kc_fvqmrBrK5FVuMNhhg0kxxm75TuIDgMQguCWzA5cweCiTkDj8Vg375Mh9vgothgZ-N-wG3PlIddHDlE9CIZzIouqOAyWPc6jXae8PM3I_doSQ?type=png)](https://mermaid.live/edit#pako:eNqdU8luwyAQ_RXEOZHbqw89VL321N5ChajBMQqLxaKoivLvHXCM3IS0lX3Aw-O9YRbmhDvLBW4xuny9ssduYC6g92diEFKdo0owLty8kyYIN1rFgpih3iqQVSnUSx2veRfQx8-9Y-OAKKXcOgY_tEvGRxIsT4PUoJrWRMpWZiGUBE0GGi2Vkr55fNgRUuwm84ThxOSlEgrablGQ3QExj8aoFB2tl0FaAwdPlRLM4qQrVNAWpzf6StEml9cuJvRfDm5SgPQKf9mSWoXyvdVUf2lmEu0tW4gg4qOT0Oaf8D1fq3Muz2hdLn_Kc_fvqmrBrK5FVuMNhhg0kxxm75TuIDgMQguCWzA5cweCiTkDj8Vg375Mh9vgothgZ-N-wG3PlIddHDlE9CIZzIouqOAyWPc6jXae8PM3I_doSQ)

- `mono_replay_real.yml`: A simple real replay pipeline that allows you to replay a recorded episode.

You must configure the dataset path and episode index in the file `mono_replay_real.yml` to set the correct
environment variables. (e.g. `PATH`, `EPISODE`). You must also configure the follower arm, retrieve the device port, and
modify the file `mono_replay_real.yml` to set the correct environment variables. (e.g. `PORT` and `CONFIG`)

```bash
cd dora-lerobot/

# If you are using a custom environment, you will have to activate it before running the command
source [your_custom_env_bin]/activate

# If you followed the installation instructions, you can run the following command
source venv/bin/activate # On Linux
source venv/Scripts/activate # On Windows bash
venv\Scripts\activate.bat # On Windows cmd
venv\Scripts\activate.ps1 # On Windows PowerShell

dora build ./robots/alexk-lcr/graphs/mono_replay_real.yml # Only the first time, it will install all the requirements if needed

dora up
dora start ./robots/alexk-lcr/graphs/mono_replay_real.yml
```

[![](https://mermaid.ink/img/pako:eNptkbFuAyEMhl_F8pzohmw3dKiydmq3UCH38N2hcoB8oCiK8u4BmkZNWwbz8_PZCPuMQzCMPcJtjS4ch5kkwduz8gDC0dFJD86yT9Vwg-gxuIKxKL_mj0kozqC1NkGobHCo4r2yP2-TXVhusUJNNQqgJnTN6BbrnF273e6g1F13jWNvlG_h_wzYbiFm53QMq002-GI8_f3Ag9FyvnFa4Sg2sZ4C_ary-GvcYHl5IWtK5861qMI088IK-yINyadC5S-Fo5zC68kP2CfJvEEJeZqxH8mt5ZSjocR7S6VNy91lY1OQl6_BtPlcrmjBlKg?type=png)](https://mermaid.live/edit#pako:eNptkbFuAyEMhl_F8pzohmw3dKiydmq3UCH38N2hcoB8oCiK8u4BmkZNWwbz8_PZCPuMQzCMPcJtjS4ch5kkwduz8gDC0dFJD86yT9Vwg-gxuIKxKL_mj0kozqC1NkGobHCo4r2yP2-TXVhusUJNNQqgJnTN6BbrnF273e6g1F13jWNvlG_h_wzYbiFm53QMq002-GI8_f3Ag9FyvnFa4Sg2sZ4C_ary-GvcYHl5IWtK5861qMI088IK-yINyadC5S-Fo5zC68kP2CfJvEEJeZqxH8mt5ZSjocR7S6VNy91lY1OQl6_BtPlcrmjBlKg)

## License

This library is licensed under the [Apache License 2.0](../../LICENSE).