import subprocess

# Define the command as a list of arguments
command = ["logger.py", "2>", ">(grep -v TF_REPEATED_DATA buffer_core)"]

# Run the command
process = subprocess.Popen(command, stderr=subprocess.PIPE, shell=True)

# Get the stderr output
stderr_output = process.stderr.read()

# Wait for the command to complete
process.wait()

# Print the stderr output
print(stderr_output.decode())
