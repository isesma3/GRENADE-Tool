Copyright (c) 2018 Uber Technologies, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

# OpenVSP Python Packages

## Background

This README is copied and modified from OpenVSP Software. This document provides guidance 
regarding installation and execution of OpenVSP Python API in ASDL's SANTA Software.

## Installation

Before installation, ensure you have installed Python via Anaconda or
Miniconda.  Conda is required for the environment management utilized in this
repository.  When installing via Anaconda or Miniconda, pip should also be
installed, but it is worth verifying that this has been done.


### OpenVSP Download
1. Download OpenVSP from its website: [OpenVSP](https://openvsp.org/download.php)
2. Extract the folder in the desired repository. (*It should not be SANTA repository*)


### Windows Installation Only

1. Open a Windows PowerShell
2. Navigate to the location of this README.md file using *cd* bash command.
3. Execute `./setup.ps1`

### Mac OS Installation (via BASH)

1. Open a terminal window and navigate to the location of this README.md file using *cd* bash command.
2. Execute `conda env create -f ./environment.yml
3. Execute `conda activate vsppytools`
4. Execute `pip install -r requirements.txt`

   Note: You can install `requirements-dev.txt` if you are going to modify the Python packages. See [the pip install documents](https://pip.pypa.io/en/stable/cli/pip_install/#cmdoption-e) for more info on installing with `-e`.

### Linux Installation (via BASH)

Linux users often use OpenVSP as installed by their packaging system in a
location such as /opt/OpenVSP.  The final step of this process required write
permissions.  You shouldn't need to do this as root, so we'll start by copying
all the files somewhere you will have write access.

1. Open a terminal window and navigate to the location of this README.md file.  If you have write permissions, continue with the MacOS instructions above.
2. Execute `cd ..; cp -r python /tmp/vsptemp; cd /tmp/vsptemp`
3. Execute `conda env create -f ./environment.yml
4. Execute `conda activate vsppytools`
5. Execute `pip install -r requirements.txt`

   Note: You can install `requirements-dev.txt` if you are going to modify the Python packages. See [the pip install documents](https://pip.pypa.io/en/stable/cli/pip_install/#cmdoption-e) for more info on installing with `-e`.

### Nota Bene

The previous installation has created a conda environment named *vsppytools*. This conda env
shall be activated anytime the user wants to use OpenVSP extension of SANTA. It is highly recommended
to use Visual Studio Code.

### Activating Conda Environment In VSCode on Windows

1. Open VSCode
2. Open a Python file (any file)
3. Check the current Python interpreter in the bottom right hand corner
4. If not 'vsppytools' is not mentionned, Click on the interpreter button
5. If 'vsppytools is mentionned, Select the corresponding Python interpreter
6. If not, Select 'Enter interpreter path' and Click on 'Find'
7. Find your 'anaconda3' folder. It should be at the Home path of your laptop (C:\Users\username)
8. Navigate to the folder '/anaconda3/envs/vsppytools/'
9. Select 'python.exe' file.

From now on, you should be able to select the right interpreter whenever you need it
directly in VSCode by clicking in the bottom right hand corner.
Users can also activate the conda environment in a terminal by running: `conda activate vsppytools`

### Documentation

OpenVSP provides documentation on its API on its website: [Documentation](https://openvsp.org/api_docs/latest/)