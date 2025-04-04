import os
from enum import Enum
import yaml
import copy
from typing import Dict

import rospy
from dynamic_reconfigure.client import Client


class Modes(Enum):
    JOINT_TASK = 0
    JOINT = 1
    TASK = 2
    

class DynamicReconfigure:
    def __init__(self, param_dict=None):
        # Create Dynamic Reconfigure Server
        # self._srv = Server(config, callback)
        # Create a Dynamic Reconfigure Client to update the parameters
        self._client = Client("compliant_controller_reconfigure_node")
        # Dictory containing the params
        self._default_params = param_dict
        self._params = copy.deepcopy(self._default_params)   
        
    def callback(self, config):
        rospy.loginfo("Reconfigured with: %s", config)
        return config
    
    def update_parameters(self, params: Dict=None):
        if params is not None:
            self._params = params     
        return self._client.update_configuration(self._params)
    
    # Setters and Getters
    @property
    def default_params(self) -> Dict:
        return copy.deepcopy(self._default_params)
    
    @default_params.setter
    def default_params(self, values: Dict):
        self._default_params = copy.deepcopy(values)
        
    @property 
    def params(self) -> Dict:
        return self._params
    
    @params.setter
    def params(self, values: Dict):
        self._params = values
    

def load_yaml_config(file_path):
    with open(file_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def flatten_config(nested_dict):
    flat_dict = {}
    for group, params in nested_dict.items():
        for key, value in params.items():
            flat_dict[key] = value
    return flat_dict

def load_default_params(config_path, file_dict):
    param_dict = {}
    for key, file_name in file_dict.items():
        config = load_yaml_config(os.path.join(config_path, file_name + ".yaml"))
        param_dict[key] = flatten_config(config)
    return param_dict


def create_reconfig_dict(gain_dict):
    reconfig_dict = {}
    for key, item in gain_dict.items():
        reconfig_dict[key] = DynamicReconfigure(item)
    return reconfig_dict


def main():
    
    rospy.init_node("mode_switcher")
    
    config_path = rospy.get_param(rospy.get_name() + "/config_path")
    mode = rospy.get_param(rospy.get_name() + "/mode")
    
    file_dict = {Modes.JOINT_TASK: "joint_task_space_params",
                 Modes.JOINT: "joint_space_params",
                 Modes.TASK: "joint_task_space_params"}
    
    # The default gains for the three modes 
    gain_dict = load_default_params(config_path, file_dict)
    
    # Reconfigure dict
    reconfig_dict = create_reconfig_dict(gain_dict)
    
    # Execute mode switch
    reconfig_dict[Modes(mode)].update_parameters()
    
    return 0


if __name__ == "__main__":
    main()
