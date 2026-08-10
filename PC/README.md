# SAFE LQR PC host

`sae_pc_hil.py` is the PC-side HIL controller for the STM32 firmware in this
repository.

## Online EAGA implementation

The online optimizer now implements the same mechanisms described in the
manuscript:

- two independently evaluated subpopulations of 10 candidates each;
- a shared, bounded elite archive containing the six best unique candidates;
- rank-dependent diffusion
  `sigma_i = sigma_base * (1 + alpha * rank_i / population_size)`;
- elite-archive-guided offspring
  `child = parent + beta * (archive_elite - parent) + N(0, sigma_i^2 I)`;
- warm starting, per-subpopulation elitism, and clipping to a bounded local
  log-weight search domain around the incumbent controller.

Candidate fitness remains the model-rollout IAE plus the saturation penalty,
and every returned gain must pass the existing stability and safety checks.
The two subpopulations are evaluated concurrently with two host threads.

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

## Test

Run the deterministic structural checks from the repository root:

```powershell
python -m unittest PC.test_eaga
```

The tests verify the two-subpopulation layout, elite-archive capacity and
ordering, monotonically rank-scaled diffusion, and finite optimizer output.
