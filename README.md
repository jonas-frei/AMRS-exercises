# Description

Basic EEROS project for the Beaglebone Blue with a minimal control system, safety system and sequencer. This serves as a template to start working on a new project.

# Overview

**Folder structure**
```
Main project folder
|CMakeList.txt
|-src
| |*.cpp
|-include
| |*.hpp
|-config
| |HWConfigBBBlue.json
```

The main project folder contains the [eeros build scripts](https://github.com/eeros-project/eeros-build-scripts), as well as the folder template_project. In the folder template_project is all the code and the CMakeLists.txt file, which is needed to compile the application. *.cpp files are stored in the src folder and *.hpp files in the inc folder. In the config folder is the HWConfigBBBlue.json file, which is used to interface with the hardware through the HAL.

# Prerequisits

In order to be able to compile your project on the host, you need to have the [EEROS SDK installed](https://wiki.eeros.org/getting_started/install/use_on_bbb).

# How to use the template project

1. Clone the [eeros build scripts](https://github.com/eeros-project/eeros-build-scripts)

    `git clone https://github.com/eeros-project/eeros-build-scripts.git YOUR_PROJECT_NAME`

2. Modify the following lines in the file config.sh.in

    ```
    custom_application_name=YOUR_PROJECT_NAME
    custom_application_git_remote_address=https://github.com/eeros-project/eeros-template-project-bbblue.git
    ```

3. Fetch the code of the application by executing the `clone.sh` script from within the main project folder

    `./clone.sh`

4. Optionally create a new git repository and change the remote URL of the origin:

    `git remote set-url origin your/new/repo/url/YOUR_PROJECT_NAME.git`

    Change the remote address in the config.sh.in to your new address.

5. Alternatively delete the .git folder (**do NOT do this if you did step 3!**):

    `rm -rf .git`

6. Replace `template_project` with `YOUR_PROJECT_NAME` in the following files and folders:

    - Folder `template_project` in `main project folder`

    - File `deploy.txt`, line `./build-armhf/template_project/template_project`

    - File `CMakeLists.txt`, line `project(template_project)`

7. Compile the project by executing the `make.sh` script from within the main project folder

    `./make.sh`

8. Deploy your project to the target

    - Connect the Beaglebone Blue Board to the host

    - Execute the `deploy.sh` script from within the main project folder

        `./deploy.sh`

9. SSH into the target and run the application.

10. Start working on your own project by modifying the code, creating new blocks and sequences, ...
