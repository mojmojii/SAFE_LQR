# SAFE LQR PC host

`sae_pc_hil.py` is the PC-side HIL controller for the STM32 firmware in this
repository.

## Requirements

```powershell
python -m pip install numpy scipy pyserial
```

## Run

Set `MODE` near the top of `sae_pc_hil.py`:

- `selftest`: run the local simulation without hardware.
- `identify`: collect pulse-response data and identify model parameters.
- `real`: communicate with the STM32 over the configured serial port.

Then run:

```powershell
python .\PC\sae_pc_hil.py
```

For hardware use, configure `COM_PORT` first. Keep `ALLOW_DOWNLINK = False`
until the model, gain signs, limits, and firmware protocol have been verified.
