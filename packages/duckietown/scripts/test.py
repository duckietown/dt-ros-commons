#!/usr/bin/env python3

from duckietown_utils import yaml_load_file

if __name__ == '__main__':
    fn = '/data/config/calibrations/camera_intrinsic/autobot03.yaml'
    print(yaml_load_file(fn, plain_yaml=True))
