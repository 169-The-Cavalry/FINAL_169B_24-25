FINAL_169B_24-25 Repository

This repository serves as the central hub for our robot code, autonomous strategies, and team documentation as we design, build, and program our robot for the upcoming competitions. It is organized to ensure that all team members can collaborate effectively and have access to the necessary tools to contribute to the project.

This is a GitHub Codespace

Our development environment is hosted on GitHub Codespaces, which ensures that all extensions, console permissions, and formatting are preconfigured and synchronized. By using GitHub Codespaces, we create a consistent and uniform environment for every team member, reducing setup time and avoiding compatibility issues.

Key Features of the Codespace:

Cloud-based, no need for local setup
Shared development environment
All tools and extensions preconfigured
Access to the repository and all team resources
Requirements for Branching or Editing This Workspace Locally:
If you prefer to edit this workspace on your local machine, make sure to follow these guidelines and install the necessary extensions:

Extensions to Install:

VEXCode VSCODE Extensions – Required to interface with the VEX robotics platform.
Python – For running and debugging Python scripts.
Python Debugger – To debug Python code.
Pylance – Enhances the Python development experience with linting, IntelliSense, and type checking.
Symbolizer for VEX V5 – To ensure the correct symbolization and debugging of VEX V5 specific code.
Console Permissions (For Self-Made Compiler):


To ensure that your custom scripts and files are executable from the terminal, follow these steps:

1. Open the terminal in VS Code.
2. Navigate to the directory containing the script by running: cd /workspaces/FINAL_169B_24-25/src/
3. Run the following command to give the script execution permissions: chmod +x bundle.sh

   
Use Notes:

1. Uploading through the Brain (More Reliable):

It’s recommended to upload the compiled code to the VEX Brain through the VEXCode interface, as this is a more reliable method for deployment.

3. Ignoring Problems Except for main.py:

For now, you can ignore errors in any files other than main.py. Other files are not yet compiled and do not import the VEX libraries.

5. Python VM Updates:

Regular updates are necessary for Python VM environments to ensure compatibility with the latest VEX Robotics firmware and Python tools. Please stay on top of these updates.


How to Compile the Code:

To compile the code for deployment, follow these steps in VS Code:

1. Open the Command Palette:Press Ctrl+Shift+P (Windows/Linux) or Cmd+Shift+P (Mac).

2. In the palette, type >Tasks: Run Task.

3. Run the "Bundle Python Files" Task:

4. Select the "Bundle Python Files" task from the list.

This will compile the Python code for deployment to the robot (takes 1-2 seconds).


Additional Notes:

If the bundle.sh script runs without issues, it bundles the code files together and prepares them for upload to the robot.

You can use this task every time you update or modify the code for the robot.


Future Work and Collaboration:

As we work through the competition season, the repository will continue to evolve with more autonomous strategies, code improvements, and bug fixes.

Please branch from the main branch when starting a new feature or fixing a bug.

Pull requests will be reviewed and merged to ensure that the code is stable and ready for deployment.


Repository Structure:

Here's an overview of the main files and directories you will be working with:

src/: Contains the source code for the robot.
docs/: Documentation on the robot design, competition strategies, and code usage.
tests/: (Optional) Unit tests and testing framework for the code.
bundle.sh: Script to bundle and prepare code for uploading to the robot.
