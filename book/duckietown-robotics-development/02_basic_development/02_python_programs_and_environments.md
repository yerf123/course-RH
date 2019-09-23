# Python programs and environments {#python-programs-environments status=ready}

Excerpt: Learn how to setup a Python project and associated tools.

We assume you are already quite comfortable with Python. Nevertheless, when you work with big and complex projects, there are some subtleties that you must consider and some handy tools that can make your life easier. Let's take a look at some of these now.

<minitoc/>

<!-- !!! UPDATE THIS !!! -->
<div class='requirements' markdown='1'>
  Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup)

  Results: Developer knowledge of Python
</div>

## Define a basic project structure  {status=ready}

In Duckietown, everything runs in Docker containers. All you need in order to run a piece of software in Duckietown is a Duckietown-compliant Docker image with your software in it.

A boilerplate is provided by the following [repository](https://github.com/duckietown/template-basic).

The repository contains a lot of files, but do not worry, we will analyze them one by one.

First of all, you will need to make a copy of the template in your own GitHub account. To do so, go to the repository and click on the fork button.

<figure>
    <img style='width:10em' src="images/fork.png"/>
</figure>

Now that you have a copy of the template, you can create new repositories based off of it. In order to do so, go to [GitHub.com](https://github.com/) and click on the button `[+]` at the top-right corner of the page and then click on *New Repository*.

<figure>
    <img style='width:10em' src="images/new_repo.png"/>
</figure>

You will see a page that looks like the following:

<figure>
    <img style='width:10em' src="images/create_a_repo.png"/>
</figure>

In the section *Repository template*, select *YOUR_NAME/template-basic*. Pick a name for your repository (say `my-program`) and press the button *Create repository*. Note, you can replace `my-program` with the name of the repository that you prefer, make sure you use the right name in the instructions below.

This will create a new repository and copy everything from the repository `template-basic` to your new repository. You can now open a terminal and clone your newly created repository.

    laptop $ git clone https://github.com/YOUR_NAME/my-program
    laptop $ cd my-program

NOTE: Replace `YOUR_NAME` in the link above with your GitHub username.

The repository contains already everything you need to create a Duckietown-compliant Docker image for your program. The only thing we need to change before we can build an image from this repository is the repository name in the file Dockerfile. Open it using the text editor you prefer and change the first line from:

'''
ARG REPO_NAME="<REPO_NAME_HERE>"
'''

to

'''
ARG REPO_NAME="my-program"
'''

Save the changes. We can now build the image, even though there is not going to be much going on inside it until we place our code in it. To do that, open a terminal and move to the directory created by the `git clone` instruction above. Run the following command:

    laptop $ dts devel build -f --arch amd64

If you correctly installed Docker and the `duckietown-shell`, you should see a long log that ends with something like the following:

<figure>
    <img style='width:10em' src="images/dts_devel_build.png"/>
</figure>

You can now run your container by executing the following command.

    laptop $ docker run -it --rm duckietown/my-program:v1-amd64

This will show the following message:

    container $ The environment variable VEHICLE_NAME is not set. Using '774a2521b42e'.
    container $ > Adding /code/my-program to PYTHONPATH
    container $ > Adding /code/dt-commons to PYTHONPATH
    container $ Activating services broadcast...
    container $ Done!
    container $
    container $ This is an empty launch script. Update it to launch your application.
    container $
    container $ Deactivating services broadcast...
    container $ Done!

Congratulations! You just built and run your first Duckietown-compliant Docker image.


## Run a basic program on your Laptop {status=ready}

Now that we know how to build a Docker image for Duckietown, let’s put some code in one of them.

We will see how to write a simple Python program, but any language should do it.

Open a terminal and go to the directory `my-program` created above. In Duckietown, Python code must belong to a Python package. Python packages are placed inside the directory code in my-program. Let go ahead and create a directory called `my_package` inside code.

    laptop $ mkdir -p ./code/my_package

A Python package is simply a directory containing a special file called `__init__.py`. So, let’s turn that `my_package` into a Python package.

    laptop $ touch ./code/my_package/__init__.py

Now that we have a Python package, we can create a Python script in it. Use your favorite text editor to create the file ``./code/my_package/my_script.py` and place the following code inside it.

'''
message = "Hello World!"
print(message)
'''

We now need to tell Docker we want this script to be the one executed when we run the command `docker run`. In order to do so, open the file `launch.sh` and replace the line

'''
echo "This is an empty launch script. Update it to launch your application."
'''

with the line

'''
dt_exec python3 -m "my_package.my_script"
'''

NOTE: Always prepend dt_exec to the main command in launch.sh.

If you are curious about why that is important, we can tell you that it helps us deal with an interesting problem called “The zombie reaping problem” (more about this in this [article](https://blog.phusion.nl/2015/01/20/docker-and-the-pid-1-zombie-reaping-problem/)).

Let us now re-build the image

    laptop $ dts devel build -f --arch amd64

and run it

    laptop $ docker run -it --rm duckietown/my-program:v1-amd64

This will show the following message:
    container $ The environment variable VEHICLE_NAME is not set. Using '774a2521b42e'.
    container $ > Adding /code/my-program to PYTHONPATH
    container $ > Adding /code/dt-commons to PYTHONPATH
    container $ Activating services broadcast...
    container $ Done!
    container $
    container $ Hello World!
    container $
    container $ Deactivating services broadcast...
    container $ Done!

Congratulations! You just built and run your own Duckietown-compliant Docker image.


## Run a basic program on your Duckiebot {status=ready}

Now that we know how to package a piece of software into a Docker image for Duckietown, we can go one step further and write code that will run on the robot instead of our laptop.

This part assumes that you have a Duckiebot up and running with hostname `MY_ROBOT`. Of course you don’t need to change the hostname to `MY_ROBOT`, just replace it with your robot name in the instructions below. You can make sure that your robot is ready by executing the command

    laptop $ ping MY_ROBOT.local

If we can ping the robot, we are good to go.

Before we start, we need to configure the Duckiebot to accept new code. This is necessary because the Duckiebot by defaults runs only code released by the Duckietown community. In order to configure the robot to accept custom code, run the following command,

    laptop $ dts devel watchtower stop

NOTE: You need to do this once and the effect will be lost when the Duckiebot reboots.

Let us go back to our script file my_script.py and change it to:

'''
import os
message = "Hello from {}!".format(os.environ['VEHICLE_NAME'])
print(message)
'''

We can now modify slightly the instructions for building the image so that the image gets built directly on the robot instead of your laptop or desktop machine. Run the command

    laptop $ dts devel build -f --arch arm32v7 -H MY_ROBOT.local

As you can see, we changed two things, one is `--arch arm32v7` which tells Docker to build an image that will run on ARM architecture (which is the architecture the CPU on the robot is based on), the second is ``-H MY_ROBOT.local` which tells Docker where to build the image.

Once the image is built, we can run it on the robot by running the command

    laptop $ docker -H MY_ROBOT.local run -it --rm --net=host duckietown/my-program:v1

If everything worked as expected, you should see the following output,

The environment variable VEHICLE_NAME is not set. Using 'MY_ROBOT'.
> Adding /code/my-program to PYTHONPATH
> Adding /code/dt-commons to PYTHONPATH
Activating services broadcast...
Done!

Hello from MY_ROBOT!

Deactivating services broadcast...
Done!

Congratulations! You just built and run your first Duckietown-compliant and Duckiebot-compatible Docker image.


## Install dependencies using package managers (e.g., apt, pip) {status=ready}

It is quite common that our programs need to import libraries, thus we need a way to install them. Since our programs reside in Docker images, we need a way to install libraries in the same image. The template provided by Duckietown supports two package managers out of the box:

     - Advanced Package Tool (apt)

     - Pip Installs Packages for Python3 (pip3)

List your apt packages or pip3 packages in the files `dependencies-apt.txt` and `dependencies-py3.txt` respectively before running the command `dts devel build`.

#### Basic NumPy program {#exercise:ex-docker-numpy}
Write a program that performs the sum of two numbers using [NumPy](https://numpy.org/). Add `numpy` to the file `dependencies-py3.txt` to have it installed in the Docker image.

<end/>

Here you go! Now you can handle pip dependencies as well!
