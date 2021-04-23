# Description

Basic EEROS project with a minimal control system, safety system and sequencer. This serves as a template to start working on a new project.

# Overview

**Folder structure**
```
Main project folder
|CMakeList.txt
|-src
| |*.cpp
|-include
| |*.hpp
```

The main project folder contains all the code and the CMakeLists.txt file, which is needed to compile the application. *.cpp files are stored in the src folder and *.hpp files in the inc folder.

# Prerequisits

In order to be able to compile your project, you need to have [EEROS installed](https://wiki.eeros.org/getting_started/install). Follow the respective installation instructions depending on whether you want to use EEROS on your host, the beaglebone blue or the CB20 board.

# How to use the template project

**NOTE: the template project does not support the cb20 board yet, even though the description already mentions it!**

1. Clone the [eeros build scripts](https://github.com/eeros-project/eeros-build-scripts)

    `git clone https://github.com/eeros-project/eeros-build-scripts.git YOUR_PROJECT_NAME`

2. Change into the new directory. Depending on whether you want to use EEROS on your host, the beaglebone blue or the CB20 board checkout the respective branch with

    ```
    git checkout host
    git checkout sdk_bbb
    git checkout sdk_cb20
    ```

3. Modify the following lines in the file config.sh.in

    ```
    custom_application_name=YOUR_PROJECT_NAME
    custom_application_git_remote_address=https://github.com/eeros-project/eeros-template-project-bbblue.git
    custom_application_git_version=host/bbblue/cb20     # choose the correct one for your application
    ```

4. Fetch the code of the application by executing the `clone.sh` script from within the main project folder

    `./clone.sh`

5. Optionally create a new git repository and change the remote URL of the origin:

    `git remote set-url origin your/new/repo/url/YOUR_PROJECT_NAME.git`

    Change the remote address in the config.sh.in to your new address and the version to master.

6. Alternatively delete the .git folder (**do NOT do this if you did step 3!**):

    `rm -rf .git`

7. Replace `template_project` with `YOUR_PROJECT_NAME` in the following files and folders:

    - Folder `template_project` in `main project folder`

    - File `deploy.txt`, line `./build-armhf/template_project/template_project`

    - File `CMakeLists.txt`, line `project(template_project)`

8. Compile the project by executing the `make.sh` script from within the main project folder

    `./make.sh`

9. If you are working on the beaglebone blue or CB20, deploy your project to the target

    - Connect the Beaglebone Blue/CB20 Board to the host

    - Execute the `deploy.sh` script from within the main project folder

        `./deploy.sh`

10. If you are working on the beaglebone blue or CB20, SSH into the target and run the application. Otherwhise directly run the application on the host.

11. Start working on your own project by modifying the code, creating new blocks and sequences, ...
