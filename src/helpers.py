# import yaml
import os
import sys
import io

# get current script path as anchor
def get_script_path():
    return os.path.dirname(os.path.realpath(sys.argv[0]))

# def parse_config(path): 
#     with open(path, 'r') as file: 
#         config = yaml.load_all(file, yaml.FullLoader)
#     return config