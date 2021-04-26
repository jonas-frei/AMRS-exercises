# Description

Basic EEROS project with a minimal control system, safety system and sequencer implementation. This may serve as a template to start working on a new project.

# Overview

**Folder structure**
```
YOUR_PROJECT_NAME
|CMakeList.txt
|-config
| |*.json
|-inc
| |*.hpp
|-src
| |*.cpp
```

The main project folder contains the CMakeLists.txt file, which is needed to compile the application, as well as the folders config, inc and src.

In the config folder *.json files are stored. These files are needed to interface with the hardware through the HAL and are therefore hardware specific. *Note: at the moment only the hardware config file for the beaglebone blue is provided*

In the inc folder *.hpp files are stored. Declare your functions and classes in these header files. (Custom) blocks, sequences and steps can also be directly defined in these files.

In the src folder *.cpp files are stored. In there define your functions and classes previously declared but not yet defined in the header files.

# Prerequisits

In order to be able to compile your project, you need to have [EEROS installed](https://wiki.eeros.org/getting_started/install). Follow the respective installation instructions depending on whether you want to use EEROS on your host, the beaglebone blue or the CB20 board.

# How to use the template project

*Note: in the following replace `YOUR_PROJECT_NAME` with whatever you want to name your project*

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
    ```

4. Fetch the code of the application by executing the `clone.sh` script

    `./clone.sh`
    
    This will create a new directory inside the `YOUR_PROJECT_NAME` directory, which is also called `YOUR_PROJECT_NAME`

5. If you want to keep working with git, create a new git repository and:

    - Change into the cloned project directory with `cd YOUR_PROJECT_NAME`
    
    - Change the remote URL of the origin (**make sure that you are in the subdirectory `YOUR_PROJECT_NAME/YOUR_PROJECT_NAME` before you execute this step**):

        `git remote set-url origin your/new/repo/url/YOUR_PROJECT_NAME.git`

    - Change back into the main follder with `cd ..` and change the remote address in the config.sh.in to your new remote address.

6. Replace `template_project` with `YOUR_PROJECT_NAME` in the following file `CMakeLists.txt`, line `project(template_project)`

7. Compile the project by executing the `make.sh` script from within the main project folder

    `./make.sh`

8. If you are working on the beaglebone blue or CB20, deploy your project to the target. As a guideline look at the chapter [Deploying](https://wiki.eeros.org/getting_started/deploy) on the EEROS wiki 

9. If you are working on the beaglebone blue or CB20, SSH into the target and run the application. Otherwhise directly run the application on the host.

10. Start working on your own project by modifying the code, creating new blocks and sequences, ...
