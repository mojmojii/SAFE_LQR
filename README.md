# SAFE-LQR

Embedded STM32F427 firmware and a Python host implementation for
sensor-triggered, model-based online LQR tuning on a ball-and-beam platform.

## Repository layout

- `PC/`: host-side model rollout, EAGA optimization, safety screening, UART
  protocol, self-test, and algorithm tests.
- `StateMachine/`: embedded LQR, monitoring, gain switching, and fallback logic.
- `Core/`, `Drivers/`, `Middlewares/`: STM32F427 firmware support code.

Start with [PC/README.md](PC/README.md) for dependencies, optimizer details,
test commands, and hardware-operation safeguards.

The committed host configuration keeps `ALLOW_DOWNLINK = False`. Hardware
downlink must be enabled only after independently verifying the plant model,
gain signs, actuator limits, serial protocol, and firmware safety behavior.
