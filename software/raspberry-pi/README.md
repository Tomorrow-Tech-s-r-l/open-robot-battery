# Raspberry Pi Examples

Helper scripts for validating the Open Robot Battery hardware with a Raspberry Pi before deploying the ROS 2 stack.

## Scripts

- `raspi_ez_pd_demo.py`: Polls the EZ-PD BCR status register over I²C and prints the bitmask once per second.

## Usage

```bash
pip install smbus2
python3 software/raspberry-pi/raspi_ez_pd_demo.py
```

Run the commands from the repository root so the relative paths resolve correctly.
