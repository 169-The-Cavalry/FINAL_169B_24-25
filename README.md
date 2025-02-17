# FINAL_169B_24-25 Repository

This repository serves as the central hub for our robot code, autonomous strategies, and team documentation as we design, build, and program our robot for the upcoming competitions. It is organized to ensure that all team members can collaborate effectively and have access to the necessary tools to contribute to the project.

---

## This is a GitHub Codespace

Our development environment is hosted on GitHub Codespaces, which ensures that all extensions, console permissions, and formatting are preconfigured and synchronized. By using GitHub Codespaces, we create a **consistent and uniform environment** for every team member, reducing setup time and avoiding compatibility issues.

### Key Features of the Codespace
- **Cloud-based:** No need for a local setup.
- **Shared development environment:** All team members access the same tools and configurations.
- **Preconfigured:** All necessary extensions, settings, and console permissions are in place.
- **Always updated:** Automatic updates ensure consistency across the team.

---

## Requirements for Branching or Editing This Workspace Locally

If you prefer to edit this workspace on your **local machine**, follow these guidelines and install the necessary extensions.

### Extensions to Install
1. **VEXCode VSCODE Extensions** – Required to interface with the VEX robotics platform.
2. **Python** – For running and debugging Python scripts.
3. **Python Debugger** – To debug Python code.
4. **Pylance** – Enhances the Python development experience with linting, IntelliSense, and type checking.
5. **Symbolizer for VEX V5** – Ensures correct symbolization and debugging of VEX V5 specific code.

### Console Permissions (For Self-Made Compiler)
To ensure your custom scripts and files are executable from the terminal:

1. Open the **terminal** in VS Code.
2. Navigate to the directory containing the script:`cd /workspaces/FINAL_169B_24-25/src/`
3. Run the following command to grant execution permissions:`chmod +x bundle.sh`

### Use Notes:
#### Uploading Through the Brain (More Reliable)
It’s recommended to upload the compiled code to the VEX Brain via the VEXCode interface, as this method has proven to be more reliable.

---

### Ignoring Problems Except for `main.py`
For now, ignore errors in files other than `main.py` since other files are not yet compiled and do not import VEX libraries.

---

### Python VM Updates
Regular updates are necessary for Python VM environments to ensure compatibility with the latest VEX Robotics firmware and tools. Please ensure you are keeping these updated.

---

### How to Compile the Code
Follow these steps in VS Code to compile the code for deployment:

1. **Open the Command Palette:**
   - Press `Ctrl+Shift+P` (Windows/Linux) or `Cmd+Shift+P` (Mac).
   - Type `>Tasks: Run Task` in the Command Palette.
2. **Run the "Bundle Python Files" Task:**
   - Select the **"Bundle Python Files"** task from the list.
   - This task compiles the Python code for deployment to the robot (this process takes about 1-2 seconds).
3. **Additional Notes:**
   - The `bundle.sh` script bundles the code files together, preparing them for upload.
   - Use this task each time you update or modify the code for the robot.

---

### Future Work and Collaboration
- **As we progress through the competition season,** this repository will evolve with new autonomous strategies, code improvements, and bug fixes.
- **Branching:** Please branch off from the `main` branch when starting new features or fixes.
- **Pull Requests:** All changes should be submitted as pull requests for review to ensure stability and readiness for deployment.
- **Documentation:** Update this README and our documentation as new procedures or tools are added.

---

### Repository Structure
- **src/**: Contains the source code for the robot.
- **docs/**: Documentation on robot design, competition strategies, and code usage.
- **tests/**: (Optional) Unit tests and testing frameworks.
- **bundle.sh**: A script to bundle and prepare the code for uploading to the robot.
