# docker_rosdep

This repo showcases how to use `rosdep` to install arbitrary packages, making docker ROS packaging easier and more maintainable. It also shows the most basic features of using custom rules with rosdep

This targets `ros:noetic` containers. Rosdep distributions might be different in other installs


<!-- vim-markdown-toc GFM -->

* [The problem](#the-problem)
* [Repo contents](#repo-contents)
* [Managing dependencies with rosdep](#managing-dependencies-with-rosdep)
* [Dependencies for different contexts](#dependencies-for-different-contexts)
    * [ROS packages](#ros-packages)
    * [APT packages](#apt-packages)
    * [Python packages](#python-packages)
    * [JavaScript packages](#javascript-packages)
    * [Source installs](#source-installs)
* [What's so great about all this](#whats-so-great-about-all-this)
* [Diving deeper](#diving-deeper)

<!-- vim-markdown-toc -->

## The problem 

There are many ways to dockerize ROS packages. Don't get too caught on on the names, as I'm improvising them, but here's some examples:

- Package level: package includes a Dockerfile which, when run, places the package in a workspace already inside the container.
- Multipackage level: a set of packages including a Dockerfile which, when run, places them in a workspace already inside the container. This is the case of this repo.
- Workspace level: a `catkin_ws` directory structure including a Dockerfile which, when run, places the workspace inside a custom OS - typically a specific ROS distro. 

Dockerfiles for these three use-cases can be radically different, as they present issues with:

- Compilation: 
    - Are the compiled files kept between different `docker run`? 
    - Is the `catkin_ws/build` dir preserved? 
    - Is this relevant at all (compiled vs interpreted languages)?
- Dependencies: 
    - Do all your packages share dependencies?
    - Is it more convenient to maintain a package OR a multipackage Dockerfile?
    - How easy is it to run many Docker images, even if it is more convenient to dockerize by package?

This repo focuses on making your life easier regarding dependency handling by using `rosdep` with arbitrary, non-ROS packages.


## Repo contents

This repo packages two separate demo ROS packages:

- `my_python_pkg`: A simple python rostopic talker-listener 
- `my_js_pkg`: A simple JS node that lifts a local website
- `my_source_pkg`: A simple package carrying a dependency installed through a custom `bash` script
- `my_utils_pkg`: A simple tmux wrapper over the other package installed through `apt`

These are dockerized at package and multipackage level *at the same time*, their dependencies managed by custom rules in `rosdep.yaml` files. Notice:

- Dockerfiles are almost equal and free of dependency installs
- The actual dependency installation is made by an user call to `rdinstall.sh`, which does some additional stuff

Do run and test them. They are all `ros:noetic` base images with extra packages installed via `rosdep`. Keep reading for implementation details on how this is achieved. If you need a refresher, docker `compose` is explained here [docker-reminder.md](doc/docker-reminder.md).

- Run `my_js_pkg`

```
$ cd my_js_pkg
$ docker compose run my_js_pkg
# bash rdinstall.sh
# rosrun my_js_pkg index.js
```

- Run `my_python_pkg`

```
$ cd my_python_pkg
$ docker compose run my_python_pkg
# bash rdinstall.sh
# roslaunch my_python_pkg my_python_pkg.launch
```

- Run `my_source_pkg`

```
$ cd my_source_pkg
$ docker compose run my_source_pkg
# bash rdinstall.sh
# rosrun my_source_pkg pacvim_bringup.sh
```

- Run `my_utils_pkg`

```
$ cd my_utils_pkg
$ docker compose run my_utils_pkg
# bash rdinstall.sh
# rosrun my_utils_pkg standalone.yml
```

- Run all packages together from upper dir image:

```
$ docker compose run all_my_packages
# bash rdinstall.sh
# rosrun my_utils_package tmuxinator.yml
```



## Managing dependencies with rosdep

Rosdep is a tool to help you manage dependencies, typically within the ROS environment. 

Whenever you create a package you need to specify its dependencies in `package.xml`, then you can easily install them with `rosdep`. Package names in this file don't match common package manager aliases, but a ROS-level package index. To list a few examples, these are a few common dependencies and their sources if we'd install them via `apt`:

- `rospy`: `ros-noetic-rospy`
- `std_msgs`: `ros-noetic-std-msgs`

This is great but not all packages have a `rosdep` rule. These are defined somewhere in your ROS installation and open to contributions on the internet. You can see them and contribute new rules in the [rosdistro repository](https://github.com/ros/rosdistro/blob/master/rosdep/). In that repository, a series of `yaml` files define what aliases can install which packages and how.

Custom dependencies' rule definitions can also be written and packed alongside a package, see [rosdep.yaml in the ros wiki](http://wiki.ros.org/rosdep/rosdep.yaml). A simple rule to tell `rosdep` to install `tmux` from the apt package `tmux` can be written as:

```
# rosdep.yaml

tmux:
  ubuntu:
    tmux
```

Additional `rosdep` local files need to be shown to the active ros distribution by annotating their path in the system-level path `/etc/ros/rosdep/sources.list.d/20-default.list` and then calling `rosdep update`. You could use these commands to install any package you'd like from rules defined in your package.

```
echo 'yaml file:///catkin_ws/src/my_python_pkg/rosdep.yaml' >> /etc/ros/rosdep/sources.list.d/20-default.list
rosdep update
apt update
rosdep install --from-paths /catkin_ws/src/ -y
```

Custom rules for `rosdep` are not limited to installing through the `apt` package manager and can even carry bash instructions to perform builds and installations manually (not so easy in `ros:noetic`). Continue reading for practical shorts on installing dependencies in different contexts.



## Dependencies for different contexts

### ROS packages

ROS dependencies should already be in your ROS distribution index. No custom `rosdep.yaml` file should be neccesary. Simply write the ROS alias of the dependency in `package.xml`.
```
# In package.xml

<depend>rospy</depend>
```

This will typically draw from `ros-noetic-<pkg_name>`. In this case, rosdep will `apt install ros-noetic-rospy`.

### APT packages

Ubuntu-like dependencies can usually be installed through `apt`. Here's how you would install `tmux` through rosdep:

- Add the keyword to the package dependencies in `package.xml`.
  
```
# package.xml

<exec_depend>tmux</exec_depend>
```

- Add a custom rule for the `tmux` keyword. 
  
```
# rosdep.yaml

tmux:
  ubuntu:
    tmux
```

### Python packages

Python dependencies are installed through the `pip` package manager. Typically, you'll need to install it first, so an `apt` rule should be added for it.

- Add the keywords for `pip` and your python deps to the package dependencies in `package.xml`. Do place `pip` before other depencencies, so it gets installed first.
  
```
# In package.xml

<depend>pip</depend>
<depend>icecream</depend>
```

- Add custom rules for `pip` and your python packages. Note the syntax of the `pip` installs.
  
```
# rosdep.yaml

pip:
  ubuntu:
    python3-pip

icecream:
  ubuntu:
    pip:
      packages: [icecream]
```

### JavaScript packages

JavaScript dependencies are typically installed via `npm`, and usually reside locally in your file tree. The scarce stablishment of JS in the ROS community means there are no rigid standards on how to structure packages. Ideally, we'd recommend installing JS depencencies by using in-place bash scripting (calling `npm`) so you'll have a greater control over their project directories, but this is not supported in the rosdep packed with `ros:noetic`. In newer versions of ROS, `rosdep` allows extra installers other than `apt` or `pip`, such as `npm`. 

For now, docker `ros:noetic` standard users must resort to using the `apt` alternatives of node packages. 

- Add keywords for `nodejs`, node package manager `npm`, and dependency `express`.

```
# In package.xml

<depend>nodejs</depend>
<depend>npm</depend>
<depend>node_express</depend>
```

- Add custom rules for `npm` and `express`

```
# rosdep.yaml

# node and npm do NOT need custom rules
# npm it not neccesary unless you're installing things by hand

express:
  ubuntu:
    node-express
```

### Source installs

This category comprises custom installs via shell scripts. Noetic is trapped in between versions and cannot do this as easily as it was before. Check out `my_source_pkg` for a detailed example. Here's how a `rosdep.yaml` entry should look:

```
pacvim:
  ubuntu:
    source:
      uri: 'file:///catkin_ws/src/my_source_pkg/rosdep/pacvim.rdmanifest'
```
 
Note how `rosdep.yaml` references a `.rdmanifest` file. This file should carry three fields:

- The uri of a source tar file. If you're not packaging tar files but downloading from the internet, the common practice is to ship an empty tar file along with your package and reference its uri. 
- The installation script, with its proper shebang.
- A verification script that returns `exit 0` if the program is properly installed.

Here's an example of a `.rdmanifest` file:

```
uri: 'file:///catkin_ws/src/my_source_pkg/rosdep/empty.tar'
install-script: |
    #!/bin/bash
    apt install my_program

check-presence-script: | 
    #!/bin/bash
    ls /usr/local/bin/my_program && exit 0 || exit 1
```

## What's so great about all this

- Consistently define all your deps in the same place
- Superior package flexibility
- Super maintainable, reusable, boilerplatey `docker` assets


## Diving deeper

- Custom OS definition can be passed as arg: might help jetson vs workstation package exclusivity (both are ubuntu)
- Dependency grouping: see `time_profiler` inside [my_python_pkg/rosdep.yaml](my_python_pkg/rosdep.yaml)
- Further reading:
    - http://wiki.ros.org/rosdep/rosdep.yaml - deprecated info regarding bash scripting
    - https://github.com/ros/rosdistro/blob/master/rosdep/ -
    - https://docs.ros.org/en/independent/api/rosdep/html/ - rosdep docs
    - https://docs.ros.org/en/independent/api/rosdep/html/commands.html - rosdep docs / commands
    - https://github.com/ros-infrastructure/rosdep - rosdep repo
    - https://www.ros.org/reps/rep-0111.html - rosdep REP
    - https://ros.org/reps/rep-0112.html - source installer rosdep REP
