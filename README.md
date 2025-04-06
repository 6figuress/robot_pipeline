# Robot pipeline

This file contains all the steps from a generated texture to a .json trajectory files that can be executed on the robot

## Install

As the others repositories, you can install the differents libs with : 
```shell
uv sync
```


## Run

To run any file in this repository, you can use : 

```shell
uv run <file.py>
```

## Calibration

To perform a eye-in-hand calibration, you can first run the `takePicture.py` file, this file will allow you to save some robot poses and their linked picture.

Then, you can run the `calibrate.py` file, this will perform the calibrations using the poses presents in the `reading.csv` file, and then save the calibration results in the calibrations folder
