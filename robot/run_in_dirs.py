#! /usr/bin/env python
import subprocess
import sys
import signal
import os

# This makes multi word commands more convenient (no brackets)
localCommand = " ".join(sys.argv[1:])

targets = [
    "navigation",
    "perception",
    "hw_sensing",
    "hw_bringup"
]

print("Attempting build of these targets:")
_ = [print(target) for target in targets]

# Get this file's directory
scriptdir = os.path.dirname(os.path.abspath(__file__))

# Open subprocess: https://stackoverflow.com/questions/4798331/python-subprocess-calling-bash-script-need-to-print-the-quotes-out-too
p = []        
for target in targets:
    path = os.path.join(scriptdir, target)
    command = " ".join(["cd", path, "&&"] + localCommand.split(" "))
    print("Executing command: {}".format(command))
    
    p.append(
        subprocess.Popen([command], shell=True)
    )

## Register sigint to prematurely kill processes
def signal_handler(signal, frame):
    # Kills all running builds
    print("Interrupting running processes!")
    for entry in p:
        os.killpg(entry.pid, signal.SIGTERM)
    print('\n')
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)

#running = [ True for _ in p]

running = [ process.poll() is None for process in p ]

for run, process in zip(running, p):
    if run is False: continue
    process.communicate()

# Terminate programm if all processes have finished
print("All processes have finished. Terminating program")