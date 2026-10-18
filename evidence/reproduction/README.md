# Reproduction layout

Use Ubuntu/WSL with the retained ROS 2 Jazzy, Nav2 and Gazebo package versions.
This is a supported-environment recipe; it has not been tested on a separate
clean machine. The runner refuses changes in package/parameter snapshots rather
than silently treating a changed environment as equivalent.

The two files under artifacts/robot-failure-lab-02 are only the parameter/version
references needed by the runner. They do not replace the complete original Lab02
archive, which remains intact in the private knowledge base.

From code/robot-failure-lab/experiment03, follow README.md. Choose unused output
directories and an idle ROS domain. Run the three verification conditions before
the nine main trials. The same code/parameters must pass both stages.

source/ in the main evidence directory is the exact frozen trial source; this
reproduction copy retains it without changes. Upstream Nav2 launch snapshots
retain their original notices. No new license or publication is asserted here.
