# Description

Basic EEROS project for the Beaglebone Blue with a minimal control system, safety system and sequencer. This serves as a template to start working on a new project.

# Overview

**Folder structure**
```
Main project folder
|EEROS build scripts (clean.sh, clone.sh, config.sh.in, deploy.sh, deploy.txt, make.sh)
|-template_project
| |CMakeList.txt
| |-src
| | |*.cpp
| |-include
| | |*.hpp
| |-config
| | |HWConfigBBBlue.json
```

The main project folder contains the [eeros build scripts](https://github.com/eeros-project/eeros-build-scripts), as well as the folder template_project. In the folder template_project is all the code and the CMakeLists.txt file, which is needed to compile the application. *.cpp files are stored in the src folder and *.hpp files in the inc folder. In the config folder is the HWConfigBBBlue.json file, which is used to interface with the hardware through the HAL.

# Prerequisits

In order to be able to compile your project on the host, you need to have the [EEROS SDK installed](https://wiki.eeros.org/getting_started/install/use_on_bbb).

# How to use the template project

1. Change into the directory where you want to safe your project and clone the repository:

    `git clone https://gitlab.ost.ch/ost-robotics-buchs/eeros-template-project.git YOUR_PROJECT_NAME`

2. Change into your new project folder:

    `cd YOUR_PROJECT_NAME`

3. Optionally create a new git repository and change the remote URL of the origin:

    `git remote set-url origin your/new/repo/url/YOUR_PROJECT_NAME.git`

4. Alternatively delete the .git folder (**do NOT do this if you did step 3!**):

    `rm -rf .git`

5. Replace `template_project` with `YOUR_PROJECT_NAME` in the following files and folders:

    - Folder `template_project` in `main project folder`
    
    - File `config.sh.in`, line `custom_application_name=template_project`

    - File `deploy.txt`, line `./build-armhf/template_project/template_project`

    - File `CMakeLists.txt`, line `project(template_project)`

6. Compile the project be executing the `make.sh` script from within the main project folder

    `./make.sh`

7. Deploy your project to the target

    - Connect the Beaglebone Blue Board to the host

    - Execute the `deploy.sh` script from within the main project folder

        `./deploy.sh`

8. Start working on your own project by modifying the code, creating new blocks and sequences, ...
