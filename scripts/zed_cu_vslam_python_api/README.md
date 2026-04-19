# cuVSLAM with ZED 2i (Offline Stereo)

This folder contains a custom Python script to run **cuVSLAM** with a ZED 2i dataset in **stereo-only offline mode**, using pre-recorded images (`left/`, `right/`) and timestamps.

---

## Prerequisites

- Ubuntu 22.04
- [NVIDIA GPU with CUDA support](https://developer.nvidia.com/cuda/toolkit)
- Python 3.10
- [cuVSLAM](https://github.com/nvidia-isaac/cuVSLAM/tree/main)

---

## Install cuVSLAM

Download the appropriate wheel from NVIDIA and install:

```bash
pip install cuvslam-*.whl
```

---

## Install dependencies

```bash
pip install numpy pillow rerun-sdk
```

---

## Dataset structure

The script expects a dataset in the following format:

```
runs/<run_name>/
├── left/
├── right/
├── timestamps.txt
└── ZED2i_runtime.yaml (optional)
```

## Run the script

```bash
python3 track_zed.py
```

![CuVSLAM image api](media/cuvslam-python-api.png)

---

## Live ZED usage

For live ZED camera usage, refer to the official NVIDIA examples:

https://github.com/nvidia-isaac/cuVSLAM/tree/main/examples/zed

