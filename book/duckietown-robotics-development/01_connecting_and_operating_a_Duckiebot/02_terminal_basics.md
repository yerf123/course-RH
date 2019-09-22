# Terminal basics [MH] {#terminal-basics status=ready}

Excerpt: Learn how to use a terminal.

Working over the terminal is a skill that every roboticist-to-be needs to acquire. It enables you to work on remote agents or computers without the need for a graphical user interface (GUI) and let's you work very efficiently. Once you get the hang of it you will find out for yourself how it can make your life easier.  

<div class='requirements' markdown='1'>

Requires: [Laptop setup](+opmanual_duckiebot#laptop-setup)

Requires: [Duckietown account](+opmanual_duckiebot#dt-account)

Results: Know how to use a terminal

</div>

<minitoc/>


## Using a terminal {#using-terminal status=ready}

If you are completely new to working with a terminal, often also called console or command line, a beginners tutorial can be found [here](https://tutorials.ubuntu.com/tutorial/command-line-for-beginners#0). It makes sense to get to know the terminal very well as this will save you a lot of time along the way.

A list of commands that are frequently used can be found in the [appendix](#useful-linux-commands).

If you are looking for an extensive list of commands that can be used from the terminal, [this](https://ss64.com/bash/) is the place to look at.


## Using the duckietown-shell {#using-dt-shell status=ready}

The duckietown-shell, dt-shell or dts for short is a pure Python, easily distributable (few dependencies) utility for Duckietown.

The idea is that most of the functionality is implemented as Docker containers, and dt-shell provides a nice interface for that, so that user should not type a very long docker run command line. These functionalities range from calibrating your Duckiebot and running demos over to building the duckumentation and submitting and evaluating for AIDO. You will find the commands that you need along the way during the next steps.

If you followed all the steps in the [laptop setup](+opmanual_duckiebot#laptop-setup) nicely, you already installed the dt-shell. If not, now is the time to go back and do it. 
