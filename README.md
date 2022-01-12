# CMP9767M
These are notes to myself, especially remembering useful commands and syntax.

## Getting Started
Install instructions on LCAS wiki.
- Update using `sudo apt-get update && sudo apt-get upgrade`.
- Run `source /opt/ros/melodic/setup.bash`.
- Start simulation using `roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small`
  - If it doesn't work try `killall -9 gzserver` to ensure all simulator instances have been terminated.

## Git Version Control
- To pull from LCAS repo`git pull upstream master`. Other related commands: `git remote -v` `git branch -r` `git checkout remotes/upstream/master`.
  - From what I understand you can have many remotes with different names. `upstream` is a common one but not mandatory and not to be confused with the use of `--set-upstream` below. At some point when initialising the repo you'll need to do `git remote add <name> <url>` which connects a remote repo using `<name>` as a shortcut for <url>.  
- Contrast to `git pull origin main` to pull from my remote repo.
- Create a branch `git checkout -b <branch>` drop `-b` flag to checkout existing branch.
- If created branch locally, need to use `git push --set-upstream origin main` or more generally `git push --set-upstream <remote> <branch>`. Use the flag `-u` for `--set-upstream` instead.
- `git push origin --all` push all local branches to specified remote handy and seems to evade the need for `--set-upstream`.
  
## Understanding Workspaces and Packages
A workspace can be considered an overlaid environment ontop of the system installed one. Allows us to use packages - that we may be developing or ones from others - that aren't system installed. `catkin` is ROS's build system which essentially turns raw source code into a more useable form (I don't know the specifics). Following Marc's suggestion our file structure goal is:
```
  workspace_folder/
    src/
      CMakeLists.txt
      my_repo_folder/
        my_pkg_name/
          CMakeLists.txt
          package.xml
        ...
        another_pkg_name/
          CMakeLists.txt
          package.xml
```
- In `src/` run `catkin_init_workspace`. It will create a `CMakeLists.txt`.
- In `my_repo_folder/` initialise a git repository using `git init`.
- To create a package, set `cd` to `my_repo_folder/` and use `catkin_create_pkg` which will create a folder for the package and some files within. Use `catkin_create_pkg -h` to show help to remind of arguments syntax which will prefill `package.xml`. Alternatively don't provide arguments and populate later.
- Go to the workspace directory(necessary?). Run `catkin_make` or `catkin build`. Use either but you'll have to use the same afterwards. Marc recommends build.
- To use your own workspace run `source devel/setup.bash` (from your workspace directory). NB. can test using a command like `roscd my_pkg_name/` to see if it will shortcut you to the folder.
- Create some scripts (e.g. `my_script.py`), write some code. Then rerun `catkin build` (or make) so catkin can do its thing. These can then be run using `rosrun my_pkg_name my_script.py`. I believe future modifications to the script don't need to be rebuilt.
  

## General Linux Tips
- Command line history search
  - Press Ctrl+R and type ssh. Ctrl+R will start search from most recent command to old one (reverse-search). If you have more than one command which starts with ssh, Press Ctrl+R again and again until you find the match. [SE Answer](https://askubuntu.com/a/74633)
