#!/usr/bin/env python3
import asyncio
import subprocess
import os
from nicegui import ui
from functools import partial
from nicegui.events import ValueChangeEventArguments
import time

command_headers = [
    ["UR Driver", "XYZ Driver", "Turn Off Driver"],
    ["Realsense", "RS_Marker", "Asus Xtion", "Turn Off Sensor"],
    ["Detect Red", "Detect Colors", "Turn Off Detection"],
    ["Static Scenario", "Marker", "Red", "Multiple Color", "Turn Off Scenario"],
    ["Groot", "Rviz", "Turn Off Tools"]
]
commands = [
    ["ros2 launch ur5e_cell_manipulation manipulation_driver.launch.py ", "No Command Yet", "ls"],
    ["ros2 launch ur5e_cell_manipulation manipulation_perception.launch.py ",
        "ros2 launch ur5e_cell_manipulation manipulation_perception_aruco.launch.py ", "ls", "ls"],
    ["ros2 run color_pose_estimation color_pose_estimation_unique",
        "ros2 run color_pose_estimation color_pose_estimation", "ls"],
    ["ros2 launch ur5e_cell_manipulation manipulation_static_scenario.launch.py", "ros2 launch ur5e_cell_manipulation manipulation_aruco_scenario.launch.py",
        "ros2 launch ur5e_cell_manipulation manipulation_color_unique_scenario.launch.py", "ros2 launch ur5e_cell_manipulation manipulation_color_scenario.launch.py", "ls"],
    ["ros2 run groot Groot", "rviz2", "ls"]
]


async def run_command(event: ValueChangeEventArguments):
    """Run a command in the background and display the output in the pre-created dialog."""
    cmd = ""
    for i in range(0, 5):
        for j in range(0, len(command_headers[i])):
            if (event.value == command_headers[i][j]):
                print("Terminal run")
                cmd = commands[i][j]
                setup_cmd = 'source ~/ws_ur5e/install/setup.bash '
                full_cmd = f'gnome-terminal -- bash -c "{setup_cmd} && {cmd}"'
                ui.notify(cmd)
                break
        if cmd!="":
            break

    process = await asyncio.create_subprocess_shell(full_cmd, stdout=asyncio.subprocess.PIPE)


ui.label("Demonstrator 326").classes("text-h3 text-primary")
ui.label("Choose the components for your presentation").classes("text-h5 text-secondary")

with ui.row(): 
    ui.label("").classes("text-h6 text-muted")
with ui.row(): 
    ui.label("Driver:").classes("text-h6 text-muted")
    ui.radio(command_headers[0], value='Turn Off Driver',on_change=run_command).props('inline')
with ui.row():
    ui.label("Sensor:").classes("text-h6 text-dark")
    ui.radio(command_headers[1], value='Turn Off Sensor',on_change=run_command).props('inline')
with ui.row():
    ui.label("Detection:").classes("text-h6 text-dark")
    ui.radio(command_headers[2], value='Turn Off Detection',on_change=run_command).props('inline')
with ui.row():
    ui.label("Scenario:").classes("text-h6 text-dark")
    ui.radio(command_headers[3], value='Turn Off Scenario',on_change=run_command).props('inline')
with ui.row():
    ui.label("Tool:").classes("text-h6 text-dark")
    ui.radio(command_headers[4], value='Turn Off Tools',on_change=run_command).props('inline')
ui.run()
