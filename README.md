# Test Projects for the NVCM Framework

In order to compile the examples here the following SW and HW required:
 * Eclipse CDT
 * GNU ARM Plugins for Eclipse CDT
 * Working ARM Cross-Compiler
 * Appropriate HW with connections (example dependent)
 * A debug probe (some boards have integrated), its drivers and a working GDBServer (e.g. J-Link or OpenOCD)

## Symlinking NVCM Core

The examples here require the NVCM core sources in the "nvcm" subdirectory within the root directory of the "nvcmtests".
The easiest way to achieve this is to create a symlink.

Assuming you have downloaded the sources into c:\work\nvcmtests-master and you have downloaded the NVCM core into c:\work\nvcm-master the following command is required on Windows:

    c:\work\nvcmtests-master>mklink /d nvcm c:\work\nvcm-master

x
