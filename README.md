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
- If created branch locally, need to use `git push --set-upstream origin main` or more generally `git push --set-upstream <remote> <branch>`. Is this always true?
- `git push origin --all` push all local branches to specified remote handy.

## General Linux Tips
- Command line history search
  - Press Ctrl+R and type ssh. Ctrl+R will start search from most recent command to old one (reverse-search). If you have more than one command which starts with ssh, Press Ctrl+R again and again until you find the match. [SE Answer](https://askubuntu.com/a/74633)
