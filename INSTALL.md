# Installing Pick-and-place manipulation cross grippers without retraining

The following guidance works well for a machine with 4090 GPU, cuda 12.3, driver 545.29.06 

First, git clone this repo and `cd` into it.

    git clone https://github.com/YanjieZe/3D-Diffusion-Policy.git

---

1.create python/pytorch env

    conda remove -n dp3_mm --all
    conda create -n dp3_mm python=3.8
    conda activate dp3_mm


---

2.install torch

    # if using cuda>=12.1
    pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
    # else, 
    # just install the torch version that matches your cuda version

---

3.install dp3

    cd 3D-Diffusion-Policy/3D-Diffusion-Policy && pip install -e . && cd ..

----

4.install r3m

    cd 3D-Diffusion-Policy/third_party 
    git clone https://github.com/facebookresearch/r3m.git
    pip install setuptools==59.5.0 Cython==0.29.35 patchelf==0.17.2.0
    cd r3m && pip install -e . && cd ..

----

5.install pytorch3d

    cd 3D-Diffusion-Policy/third_party 
    

---

6.install some necessary packages

    pip install zarr==2.12.0 wandb ipdb gpustat dm_control omegaconf hydra-core==1.2.0 dill==0.3.5.1 einops==0.4.1 diffusers==0.11.1 numba==0.56.4 moviepy imageio av matplotlib termcolor hydra casadi typer==0.9.0 huggingface_hub==0.25.0 rospy rospkg franky-panda pyrealsense2 scikit-image pynput


---

