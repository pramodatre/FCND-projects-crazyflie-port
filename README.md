# Setup

## Software
Clone the repository

```shell
git clone https://github.com/pramodatre/FCND-projects-crazyflie-port.git
```

Create a virtual environment and activate

This project is tested to work with python 3.6.8 and I recommend using `pyenv` to manage your python versions if you are using MacOS.
```shell
python3 -m venv fcnd
source fcnd/bin/activate
```

Install all dependencies
```python
pip install git+https://github.com/udacity/udacidrone.git
```

## Hardware
It's assumed that you have followed [instructions on setting up Crazyflie2.1](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/). Ensure you have plugged-in Crazyflie2.1 PA radio and the Crazyflie2.1 is turned ON. Also, the connection bandwidth is set to `2M`. 