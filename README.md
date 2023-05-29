# docker-rosdep

This repo showcases how to use `rosdep` to install arbitrary packages, making docker ROS packaging easier.


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
- `my_utils_pkg`: A simple tmux wrapper over the other package  

These are dockerized at package and multipackage level *at the same time*, their dependencies managed by custom rules in `rosdep.yaml` files. Notice:

- Dockerfiles are almost equal and free of dependency installs
- Some login operations are needed through a docker `entrypoint.sh`

Do run and test them. They are all `ros:noetic` base images with extra packages installed via `rosdep`. Keep reading for implementation details on how this is achieved. 

- Run `my_python_pkg`

```
$ cd my_python_pkg
$ docker compose run my_python_pkg
# bash entrypoint.sh
# roslaunch my_python_pkg my_python_pkg.launch
```

- Run `my_js_pkg`

```
$ cd my_js_pkg
$ docker compose run my_js_pkg
# bash entrypoint.sh
# rosrun my_js_pkg index.js
```

- Run `my_utils_pkg`

```
$ cd my_utils_pkg
$ docker compose run my_utils_pkg
# bash entrypoint.sh
# rosrun my_utils_pkg standalone.yml
```

- Run both packages together from upper dir image:

```
$ docker compose run all_my_packages
# bash entrypoint.sh
# rosrun my_utils_package tmuxinator.yml
```

If you need a refresher, docker `compose` and `entrypoint` are explained here [docker-reminder.md](doc/docker-reminder.md). I'm making the user (you) call "entrypoint" by hand on purpose so you can fiddle around, but normally it'd be automated.

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

Custom rules for `rosdep` are not limited to installing through the `apt` package manager and can even carry bash instructions to perform builds and installations manually. Continue reading for practical shorts on installing dependencies in different contexts.




## Dependencies for different contexts

### ROS

ROS dependencies should already be in your ROS distribution index. No custom `rosdep.yaml` file should be neccesary. Simply write the ROS alias of the dependency in `package.xml`.
```
# In package.xml

<depend>rospy</depend>
```

This will typically draw from `ros-noetic-<pkg_name>`. In this case, rosdep will `apt install ros-noetic-rospy`.

### C/C++

Ubuntu C/C++ dependencies can usually be installed through `apt`. Here's how you would install `tmux` through rosdep:

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

### Python

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

### JavaScript

JavaScript dependencies are typically installed via `npm`, and usually reside locally in your file tree. The scarce stablishment of JS in the ROS community means there are no rigid standards on how to structure packages. Ideally, we'd recommend installing JS depencencies by using in-place bash scripting (calling `npm`) so you'll have a greater control over their project directories, but this is not supported in the rosdep packed with `ros:noetic`. In newer versions of ROS, `rosdep` allows extra installers other than `apt` or `pip`, such as `npm`. 

For now, we must resort to using the `apt` alternatives of node packages. 

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

express:
  ubuntu:
    node-express
```

### In-place bash

**No longer supported as of ROS Fuerte.**

Noetic is trapped midway and is quite limited in this regard. 

Great use cases for using in-place bash scripting would be:
- Cloning and building git repositories
- Downloading and installing `deb` files from the internet
- Carefully handling paths when installing packages locally:

```
# this does not work
express:
  ubuntu: | 
    roscd my_js_pkg && npm i express
```

## What's so great about all these

- Readably define all your deps in the same place
- Superior package flexibility
- Super maintainable, reusable, boilerplatey `docker` assets


## Diving deeper

- Custom OS definition can be passed as arg: might help jetson vs workstation package exclusivity
- Dependency grouping: see `time_profiler` inside [my_python_pkg/rosdep.yaml](my_python_pkg/rosdep.yaml)
- Further reading:
    - http://docs.ros.org/en/independent/api/rosdep/html/commands.html
    - http://wiki.ros.org/rosdep/rosdep.yaml
    - https://github.com/ros/rosdistro/blob/master/rosdep/

## Contact

Pablo Santana -> psantana@catec.aero