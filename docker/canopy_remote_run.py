#! /usr/bin/python

import yaml
import sys
import os
from collections import OrderedDict

ROS_RELEASE = "indigo"


def install_ros_packages(package):
    install_template = "sudo apt-get install ros-{}-{}"
    install_line = install_template.format(ROS_RELEASE, package)
    os.system(install_line)


build_order = OrderedDict([
    ("before_install", os.system),
    ("ros_install", install_ros_packages),
    ("install", os.system),
    ("before_script", os.system),
    ("script", os.system),
    ("after_script", os.system),
    ("before_deploy", os.system),
    ("deploy", os.system),
    ("after_deploy", os.system)])


def parse_yaml(filename):
    with open(filename) as f:
        return yaml.load(f.read())


def run_workflow(filename):
    build_dict = parse_yaml(filename)
    for key in build_order:
        func = build_order[key]
        if key in build_dict.keys():
            for proc in build_dict[key]:
                func(proc)


if __name__ == "__main__":
    filename = sys.argv[1]
    run_workflow(filename)
